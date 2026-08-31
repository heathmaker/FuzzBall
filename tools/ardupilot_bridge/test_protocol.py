#!/usr/bin/env python3
"""Stand-alone sanity check for ardupilot_bridge's UDP/JSON protocol,
independent of a real ArduPilot SITL instance: launches the compiled
controller, plays the role of the shim by sending synthetic "state"
packets, and checks the setpoint replies behave sensibly -- velocity
points at the current waypoint, speed tapers approaching the final one
(where there's nowhere left to advance to) but not at intermediate ones
(where it should immediately head for the next waypoint at cruise speed).

This exists because the actual MAVLink plumbing is owned by pymavlink
(see mavlink_shim.py) and isn't worth re-testing here; what's worth
covering without needing a full SITL install is the controller's own
guidance logic and its handling of the wire protocol it shares with the
shim.

Usage: python3 test_protocol.py [--bridge-path PATH] [--port N]
"""
import argparse
import json
import math
import socket
import subprocess
import sys
import time

WAYPOINTS = [
    (0.0, 0.0, -10.0),
    (40.0, 0.0, -10.0),
    (40.0, 40.0, -10.0),
    (0.0, 40.0, -10.0),
    (0.0, 0.0, -10.0),
]


def send_state(sock, addr, pos, mode="GUIDED", armed=True):
    state = {
        "type": "state", "t": 0.0, "armed": armed, "mode": mode,
        "pos": {"n": pos[0], "e": pos[1], "d": pos[2]},
        "vel": {"n": 0.0, "e": 0.0, "d": 0.0},
        "yaw": 0.0,
    }
    sock.sendto(json.dumps(state).encode(), addr)
    data, _ = sock.recvfrom(4096)
    return json.loads(data)


def speed_of(vel):
    return math.sqrt(vel["n"] ** 2 + vel["e"] ** 2 + vel["d"] ** 2)


def run_checks(addr):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(3.0)
    failures = []

    # 1. Far from waypoint 0 (directly "up" from the origin): expect a
    # cruise-ish speed and a climb (negative NED "d" velocity).
    reply = send_state(sock, addr, (0.0, 0.0, 0.0))
    vel = reply["vel"]
    speed = speed_of(vel)
    print(f"[far from wp0]  vel={vel}  speed={speed:.2f}")
    if vel["d"] >= 0:
        failures.append("expected climb (negative d velocity) when below waypoint 0")
    if not (1.5 < speed < 6.5):
        failures.append(f"expected a cruise-ish speed far from target, got {speed}")

    # 2. Drive through waypoints 1..(N-2): once within arrival tolerance
    # of a waypoint that still has a next one, the controller should
    # immediately point at that next one, not linger on the one just
    # reached (a real regression this test caught: the controller used to
    # advance its target index but compute that cycle's velocity from the
    # stale, already-reached target).
    for i in range(1, len(WAYPOINTS) - 1):
        prev = WAYPOINTS[i - 1]
        pos = [prev[0] + 0.5, prev[1], prev[2]]  # within arrival tolerance of `prev`
        reply = send_state(sock, addr, tuple(pos))
        vel = reply["vel"]
        target = WAYPOINTS[i]
        direction = [target[j] - pos[j] for j in range(3)]
        dlen = math.sqrt(sum(d * d for d in direction)) or 1.0
        direction = [d / dlen for d in direction]
        vlen = speed_of(vel) or 1.0
        vdir = [vel["n"] / vlen, vel["e"] / vlen, vel["d"] / vlen]
        dot = sum(direction[k] * vdir[k] for k in range(3))
        print(f"[approaching wp{i}] pos={pos} vel={vel} alignment={dot:.2f}")
        if dot < 0.9:
            failures.append(f"velocity not pointing at waypoint {i} (alignment {dot:.2f})")

    # 3. Approaching the *last* waypoint (nowhere left to advance to):
    # speed should taper down to a slow creep instead of snapping to
    # cruise speed the way step 2's intermediate waypoints did.
    last, second_last = WAYPOINTS[-1], WAYPOINTS[-2]
    far_pos = [second_last[0] + 0.5, second_last[1], second_last[2]]
    speed_far = speed_of(send_state(sock, addr, tuple(far_pos))["vel"])
    near_pos = [last[0] + 0.1, last[1], last[2] + 0.1]
    speed_near = speed_of(send_state(sock, addr, tuple(near_pos))["vel"])
    print(f"[approaching last wp] far_speed={speed_far:.2f}  near_speed={speed_near:.2f}")
    if not (speed_near < speed_far):
        failures.append("expected speed to taper down approaching the final waypoint")
    if not (0.2 < speed_near < 1.2):
        failures.append(f"expected a creep speed near the final target, got {speed_near}")

    return failures


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bridge-path", default="build/examples/ardupilot_bridge")
    parser.add_argument("--port", type=int, default=6011)
    args = parser.parse_args()

    proc = subprocess.Popen([args.bridge_path, str(args.port)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)  # let it bind its socket
    try:
        failures = run_checks(("127.0.0.1", args.port))
    finally:
        proc.terminate()
        proc.wait(timeout=5)

    if failures:
        print("\nFAILURES:")
        for f in failures:
            print(" -", f)
        sys.exit(1)
    print("\nAll protocol sanity checks passed.")


if __name__ == "__main__":
    main()
