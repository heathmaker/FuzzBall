#!/usr/bin/env python3
"""Stand-alone sanity check for ardupilot_swarm_bridge's per-agent UDP/JSON
protocol and flocking logic, independent of real SITL: launches the
compiled controller, plays all 3 agents' shims itself by sending synthetic
"state" packets on their respective ports, and checks the setpoint replies
behave sensibly -- goal-seeking when alone, separation when crowded, and
correct per-port isolation (agent i's reply always arrives on agent i's
own socket, never mixed up with another agent's).

Usage: python3 test_swarm_protocol.py [--bridge-path PATH] [--base-port N]
"""
import argparse
import json
import math
import socket
import subprocess
import sys
import time

NUM_AGENTS = 3


def make_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(3.0)
    return sock


def send_state(sock, addr, pos, vel=(0.0, 0.0, 0.0)):
    state = {
        "type": "state", "t": 0.0, "armed": True, "mode": "GUIDED",
        "pos": {"n": pos[0], "e": pos[1], "d": pos[2]},
        "vel": {"n": vel[0], "e": vel[1], "d": vel[2]},
        "yaw": 0.0,
    }
    sock.sendto(json.dumps(state).encode(), addr)
    data, from_addr = sock.recvfrom(4096)
    return json.loads(data), from_addr


def speed_of(vel):
    return math.sqrt(vel["n"] ** 2 + vel["e"] ** 2 + vel["d"] ** 2)


def run_checks(base_port):
    sockets = [make_socket() for _ in range(NUM_AGENTS)]
    addrs = [("127.0.0.1", base_port + i) for i in range(NUM_AGENTS)]
    failures = []

    # 1. All three agents far apart and far from the goal (40, 30, -10):
    # with no neighbors in range, each should just get a goal-seeking
    # velocity command pointed roughly at the goal.
    positions = [(0.0, 0.0, 0.0), (100.0, 100.0, 0.0), (-100.0, -100.0, 0.0)]
    for i in range(NUM_AGENTS):
        reply, from_addr = send_state(sockets[i], addrs[i], positions[i])
        assert from_addr[1] == base_port + i, f"agent {i} reply came from wrong port {from_addr}"
        vel = reply["vel"]
        speed = speed_of(vel)
        print(f"[agent {i}, isolated] pos={positions[i]} vel={vel} speed={speed:.2f}")
        if speed < 0.1:
            failures.append(f"agent {i} isolated: expected nonzero goal-seeking speed, got {speed}")

    # Agent 0 (at origin) should be commanded toward (40, 30, -10): both n
    # and e components positive, d component negative (climbing further).
    reply0, _ = send_state(sockets[0], addrs[0], positions[0])
    if not (reply0["vel"]["n"] > 0 and reply0["vel"]["e"] > 0):
        failures.append(f"agent 0 not heading toward goal: {reply0['vel']}")

    # 2. Place agent 0 and agent 1 right next to each other (well inside
    # the 15 m sense radius), both far from agent 2: separation should
    # push agent 0 away from agent 1 (component of velocity opposite the
    # direction to agent 1).
    pos0 = (0.0, 0.0, 0.0)
    pos1 = (0.0, 1.0, 0.0)  # 1 m east of agent 0 (n, e, d order) -- well inside sense range
    pos2 = (200.0, 200.0, 0.0)
    send_state(sockets[2], addrs[2], pos2)  # register agent 2 so it doesn't look "unreported"
    send_state(sockets[1], addrs[1], pos1)  # register agent 1's position before probing agent 0
    reply0_crowded, _ = send_state(sockets[0], addrs[0], pos0)
    vel0 = reply0_crowded["vel"]
    print(f"[agent 0, crowded by agent 1 at +1e] vel={vel0}")
    # Direction from agent 1 to agent 0 is -e (agent 0 should be pushed
    # that way, i.e. vel.e should trend negative relative to pure goal-seek
    # which alone would want positive e).
    if vel0["e"] >= 0:
        failures.append(f"expected separation to push agent 0 away from agent 1 (negative e), got {vel0}")

    # 3. Agent 0 already at its formation slot (goal (40, 30, -10) plus its
    # 5 m-ring offset, i.e. (45, 30, -10) for a 3-agent ring) but still
    # moving fast toward it (goalPull has collapsed to ~0 here): velocity
    # damping should command a slower velocity than the current one,
    # braking instead of carrying speed through the slot. Regression test
    # for the overshoot bug seen in live SITL testing, where a fast-moving
    # agent flew straight through its target and kept going.
    fast_vel = (3.0, 3.0, 0.0)
    reply0_arrived, _ = send_state(sockets[0], addrs[0], (45.0, 30.0, -10.0), vel=fast_vel)
    vel0_arrived = reply0_arrived["vel"]
    print(f"[agent 0, arrived at goal, fast] vel_in={fast_vel} vel_out={vel0_arrived}")
    if speed_of(vel0_arrived) >= speed_of({"n": fast_vel[0], "e": fast_vel[1], "d": fast_vel[2]}):
        failures.append(f"expected braking to reduce speed near goal, got {vel0_arrived} from {fast_vel}")

    if failures:
        print("\nFAILURES:")
        for f in failures:
            print(" -", f)
        sys.exit(1)
    print("\nAll swarm protocol sanity checks passed.")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bridge-path", default="build/examples/ardupilot_swarm_bridge")
    parser.add_argument("--base-port", type=int, default=6101)
    args = parser.parse_args()

    proc = subprocess.Popen([args.bridge_path, str(args.base_port)], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    time.sleep(0.5)
    try:
        run_checks(args.base_port)
    finally:
        proc.terminate()
        proc.wait(timeout=5)


if __name__ == "__main__":
    main()
