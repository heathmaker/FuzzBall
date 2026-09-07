#!/usr/bin/env python3
"""MAVLink <-> fuzzylib bridge shim.

Speaks real MAVLink to an ArduPilot vehicle (SITL or otherwise) using
pymavlink -- the reference implementation everyone from Mission Planner to
ArduPilot's own test suite relies on for correct wire-format framing and
per-message CRCs, which is not something worth hand-rolling. This script
owns none of the flight logic: it does the one-time handshake (wait for
heartbeat, arm, take off), then in a loop relays vehicle telemetry to the
fuzzylib controller process as a small JSON packet over a local UDP
socket, and relays that process's velocity/yaw reply back to ArduPilot as
a real SET_POSITION_TARGET_LOCAL_NED message.

Run order:
    1. Start ArduPilot SITL (e.g. via sim_vehicle.py -v ArduCopter).
    2. Start the fuzzylib controller: ./build/examples/ardupilot_bridge
    3. Start this script: python3 mavlink_shim.py
Mission Planner (or QGroundControl, or MAVProxy's own GCS output) can
connect to the same SITL instance at any point -- it just sees a normal
ArduPilot vehicle being flown in GUIDED mode; it does not talk to this
script or to the controller process at all.

SAFETY NOTE: unlike PX4's "offboard mode" (which automatically exits if
the setpoint stream stops), ArduPilot's GUIDED mode keeps flying the last
commanded velocity indefinitely if this bridge stops sending. That's fine
for SITL; on a real vehicle you would want a watchdog (e.g. re-send a
hover/zero-velocity command, or trigger RTL, if telemetry goes stale) --
not included here since this bridge is written and tested against SITL.
"""

import argparse
import json
import math
import socket
import sys
import time

from pymavlink import mavutil

# POSITION_TARGET_TYPEMASK bits (MAVLink common.xml), used to tell
# ArduPilot which fields of SET_POSITION_TARGET_LOCAL_NED to honor.
_POS_IGNORE = 0b0000000000000111  # ignore x, y, z position
_ACC_IGNORE = 0b0000000111000000  # ignore ax, ay, az
_YAW_RATE_IGNORE = 0b0000100000000000  # ignore yaw_rate
VELOCITY_AND_YAW_MASK = _POS_IGNORE | _ACC_IGNORE | _YAW_RATE_IGNORE

_EARTH_RADIUS_M = 6378137.0  # WGS84 equatorial radius


def latlon_to_local_ne(ref_lat_deg, ref_lon_deg, lat_deg, lon_deg):
    """Equirectangular approximation converting a (lat, lon) into local
    north/east meters relative to a reference point. Accurate to well
    under 1% at swarm scales (tens to hundreds of meters) -- survey-grade
    precision isn't the point; a *shared* reference is. Each vehicle's
    LOCAL_POSITION_NED is relative to its own EKF origin, so raw local
    positions from different vehicles aren't comparable at all; this is
    what makes them comparable, using only each vehicle's own GPS fix
    (no assumption that vehicles share a home location -- the thing that
    doesn't survive contact with real, physically separate hardware)."""
    ref_lat_rad = math.radians(ref_lat_deg)
    north = math.radians(lat_deg - ref_lat_deg) * _EARTH_RADIUS_M
    east = math.radians(lon_deg - ref_lon_deg) * _EARTH_RADIUS_M * math.cos(ref_lat_rad)
    return north, east


def wait_for_condition(predicate, timeout_s, poll_s=0.2, description="condition"):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(poll_s)
    print(f"WARNING: timed out waiting for {description}", file=sys.stderr)
    return False


class VehicleState:
    """Latest known telemetry, updated as MAVLink messages arrive.

    Position source depends on `common_origin`:
      - None (default, single-vehicle use): position comes straight from
        LOCAL_POSITION_NED, relative to this vehicle's own EKF origin.
      - (lat, lon, alt_m) (multi-vehicle/swarm use): position is instead
        derived from this vehicle's own absolute GPS fix
        (GLOBAL_POSITION_INT), converted into meters relative to the
        shared reference point every swarm member is given -- the only
        way positions from vehicles with independent EKF origins (i.e.
        any two real, physically separate vehicles) are comparable at
        all. Velocity always comes from LOCAL_POSITION_NED regardless:
        NED axes are globally north/east/down-aligned for every vehicle,
        so velocity needs no such conversion -- only position does.
    """

    def __init__(self, common_origin=None):
        self.pos_n = self.pos_e = self.pos_d = 0.0
        self.vel_n = self.vel_e = self.vel_d = 0.0
        self.yaw = 0.0
        self.relative_alt_m = 0.0
        self.armed = False
        self.mode = "UNKNOWN"
        self.have_position = False
        self.common_origin = common_origin

    def absorb(self, msg, mav):
        msg_type = msg.get_type()
        if msg_type == "LOCAL_POSITION_NED":
            self.vel_n, self.vel_e, self.vel_d = msg.vx, msg.vy, msg.vz
            if self.common_origin is None:
                self.pos_n, self.pos_e, self.pos_d = msg.x, msg.y, msg.z
                self.have_position = True
        elif msg_type == "ATTITUDE":
            self.yaw = msg.yaw
        elif msg_type == "GLOBAL_POSITION_INT":
            self.relative_alt_m = msg.relative_alt / 1000.0
            if self.common_origin is not None:
                ref_lat, ref_lon, ref_alt_m = self.common_origin
                self.pos_n, self.pos_e = latlon_to_local_ne(ref_lat, ref_lon, msg.lat / 1e7, msg.lon / 1e7)
                self.pos_d = ref_alt_m - (msg.alt / 1000.0)  # alt is MSL; down is negative-up
                self.have_position = True
        elif msg_type == "HEARTBEAT":
            self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.mode = mavutil.mode_string_v10(msg)

    def to_json(self, t):
        return {
            "type": "state",
            "t": t,
            "armed": self.armed,
            "mode": self.mode,
            "pos": {"n": self.pos_n, "e": self.pos_e, "d": self.pos_d},
            "vel": {"n": self.vel_n, "e": self.vel_e, "d": self.vel_d},
            "yaw": self.yaw,
        }


def configure_for_sitl(master):
    """Sets parameters a real vehicle would already have from its initial
    setup/calibration, but a freshly-wiped SITL instance (`-w`) starts
    without: FRAME_CLASS/FRAME_TYPE (arming refuses with "Check frame
    class and type" otherwise -- sim_vehicle.py normally injects these via
    its -f/--frame option, which launching the raw binary bypasses), and
    ARMING_CHECK/DISARM_DELAY, disabled so a simulated vehicle with no real
    accelerometer calibration or GPS-lock history isn't blocked by safety
    checks that exist for real hardware. NEVER do this on a real vehicle.
    """
    for name, value, ptype in (
        ("FRAME_CLASS", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8),  # 1 = quad
        ("FRAME_TYPE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8),  # 1 = X
        ("ARMING_CHECK", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),  # SITL only
        ("DISARM_DELAY", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT8),  # don't auto-disarm while EKF converges
    ):
        master.mav.param_set_send(master.target_system, master.target_component, name.encode(), value, ptype)
    # Drain the resulting PARAM_VALUE acks (and whatever else arrives)
    # rather than assuming a fixed delay is enough for all four to land.
    deadline = time.time() + 3.0
    while time.time() < deadline:
        master.recv_match(blocking=True, timeout=0.5)


def connect_and_prepare(connect_str, takeoff_alt_m, common_origin=None):
    print(f"Connecting to ArduPilot at {connect_str} ...")
    master = mavutil.mavlink_connection(connect_str)
    master.wait_heartbeat()
    print(f"Heartbeat received (system {master.target_system}, component {master.target_component}).")

    print("Applying SITL-only setup parameters (frame class/type, disabling ground-only safety checks) ...")
    configure_for_sitl(master)

    # Stream LOCAL_POSITION_NED and ATTITUDE at 10 Hz explicitly, rather
    # than assuming whatever default rate the vehicle happens to be
    # configured with.
    for msg_id, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10),
                        (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10),
                        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5),
                        (mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2)):
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    print("Setting mode GUIDED ...")
    master.set_mode(master.mode_mapping()["GUIDED"])

    # ArduPilot rejects both arming ("System not initialised") and takeoff
    # (MAV_CMD_NAV_TAKEOFF failing with MAV_RESULT_FAILED because the EKF
    # hasn't set its origin yet) until its EKF/AHRS finish converging --
    # confirmed via COMMAND_ACK/STATUSTEXT during bring-up: a takeoff sent
    # right after a successful arm can still fail because EKF origin isn't
    # set for another few seconds, after which ArduCopter auto-disarms from
    # sitting idle on the ground. None of this happens on a fixed schedule
    # relative to the first heartbeat, so rather than compute readiness,
    # keep retrying arm-then-takeoff as a unit (re-arming if a previous
    # attempt's idle disarm kicked in) until it actually leaves the ground.
    print("Arming and taking off (retrying until the flight controller finishes EKF/AHRS init) ...")
    state = VehicleState(common_origin=common_origin)
    last_takeoff_attempt = 0.0
    last_arm_attempt = 0.0

    def armed_and_climbing():
        nonlocal last_takeoff_attempt, last_arm_attempt
        msg = master.recv_match(type=["GLOBAL_POSITION_INT", "HEARTBEAT"], blocking=True, timeout=1.0)
        if msg is not None:
            state.absorb(msg, master.mav)
        if state.relative_alt_m > takeoff_alt_m * 0.8:
            return True
        # Throttle both retries (not every ~1s poll): re-issuing arm too
        # rapidly gave noticeably worse results in testing than a single
        # attempt every couple of seconds while ArduPilot boots.
        if not master.motors_armed():
            if time.time() - last_arm_attempt > 2.0:
                master.arducopter_arm()
                last_arm_attempt = time.time()
        # Re-send takeoff periodically (not every poll) rather than only
        # once: an early attempt can be rejected while EKF origin is still
        # settling, and re-issuing the same command once airborne is
        # harmless.
        elif time.time() - last_takeoff_attempt > 4.0:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, takeoff_alt_m)
            last_takeoff_attempt = time.time()
        return False

    wait_for_condition(armed_and_climbing, timeout_s=240, poll_s=0.5, description="arm + takeoff")
    print("Airborne -- handing control to the fuzzylib guidance controller.\n")
    return master, state


def bridge_loop(master, state, controller_addr, loop_hz):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    period = 1.0 / loop_hz
    t0 = time.time()

    while True:
        loop_start = time.time()

        # Drain every telemetry message that has arrived since last cycle.
        while True:
            msg = master.recv_match(blocking=False)
            if msg is None:
                break
            state.absorb(msg, master.mav)

        if not state.have_position:
            time.sleep(period)
            continue

        payload = json.dumps(state.to_json(loop_start - t0)).encode("utf-8")
        sock.sendto(payload, controller_addr)

        try:
            reply_bytes, _ = sock.recvfrom(4096)
        except socket.timeout:
            print("WARNING: no reply from fuzzylib controller this cycle", file=sys.stderr)
            time.sleep(period)
            continue

        setpoint = json.loads(reply_bytes.decode("utf-8"))
        vel = setpoint["vel"]
        yaw = setpoint.get("yaw", state.yaw)

        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, VELOCITY_AND_YAW_MASK,
            0, 0, 0,  # position (ignored)
            vel["n"], vel["e"], vel["d"],
            0, 0, 0,  # acceleration (ignored)
            yaw, 0.0)  # yaw, yaw_rate (ignored)

        elapsed = time.time() - loop_start
        if elapsed < period:
            time.sleep(period - elapsed)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connect", default="udp:127.0.0.1:14550",
                         help="MAVLink connection string to ArduPilot (default: %(default)s)")
    parser.add_argument("--controller-host", default="127.0.0.1")
    parser.add_argument("--controller-port", type=int, default=6001)
    parser.add_argument("--takeoff-alt", type=float, default=3.0,
                         help="Initial takeoff altitude in meters before handing off to the guidance FIS")
    parser.add_argument("--loop-hz", type=float, default=10.0)
    parser.add_argument("--common-origin", default=None,
                         help="lat,lon,alt_m shared reference point. Required for multi-vehicle use "
                              "(e.g. the swarm bridge): converts THIS vehicle's own GPS fix into the "
                              "same local frame every other swarm member also converts into, since "
                              "each vehicle's LOCAL_POSITION_NED is relative to its own independent "
                              "EKF origin and so is not otherwise comparable across vehicles. Pass "
                              "the exact same value to every shim in a swarm. Omit for single-vehicle "
                              "use (position then comes straight from LOCAL_POSITION_NED).")
    args = parser.parse_args()

    common_origin = None
    if args.common_origin is not None:
        parts = args.common_origin.split(",")
        if len(parts) != 3:
            parser.error("--common-origin must be 'lat,lon,alt_m'")
        common_origin = tuple(float(p) for p in parts)

    master, state = connect_and_prepare(args.connect, args.takeoff_alt, common_origin=common_origin)
    try:
        bridge_loop(master, state, (args.controller_host, args.controller_port), args.loop_hz)
    except KeyboardInterrupt:
        print("\nInterrupted -- commanding LAND as a SITL safety net.")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)


if __name__ == "__main__":
    main()
