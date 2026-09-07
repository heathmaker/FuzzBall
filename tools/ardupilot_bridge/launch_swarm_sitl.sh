#!/bin/bash
# Launches a multi-vehicle ArduCopter SITL swarm for the fuzzylib swarm
# bridge (examples/ardupilot_bridge/ardupilot_swarm_bridge.cpp), each
# instance from its own distinct home location -- a small staged launch
# line, the way a real multi-vehicle launch would actually be laid out
# (physically separate vehicles can't share a GPS position). Positions
# from different vehicles are only comparable once converted into one
# shared local frame; that conversion happens in mavlink_shim.py via
# --common-origin (see launch_swarm_shims.sh), not by any trick here.
#
# Run from an ArduPilot checkout:
#   NUM_AGENTS=3 bash /path/to/fuzzylib/tools/ardupilot_bridge/launch_swarm_sitl.sh
#
# Each instance N's MAVLink output lands on udp:127.0.0.1:$((14550 + N*10))
# by default (ArduPilot's standard per-instance port offset, from
# sim_vehicle.py) -- point mavlink_shim.py (see launch_swarm_shims.sh) and/or
# Mission Planner at those. Stop all instances with Ctrl+C.

set -e
NUM_AGENTS="${NUM_AGENTS:-3}"
BASE_LAT="${SWARM_BASE_LAT:-40.071374}"
BASE_LON="${SWARM_BASE_LON:--105.229195}"
BASE_ALT="${SWARM_BASE_ALT:-1600}"
LAT_SPACING_DEG="${SWARM_LAT_SPACING_DEG:-0.0001}"  # ~11 m per step, north-south launch line

pids=()
for i in $(seq 0 $((NUM_AGENTS - 1))); do
    mavlink_port=$((14550 + i * 10))
    home_lat=$(python3 -c "print(${BASE_LAT} + ${i} * ${LAT_SPACING_DEG})")
    home="${home_lat},${BASE_LON},${BASE_ALT},0"
    echo "Starting ArduCopter SITL instance $i at $home (MAVLink on udp:127.0.0.1:$mavlink_port) ..."
    Tools/autotest/sim_vehicle.py -v ArduCopter -I "$i" --custom-location="$home" -w &
    pids+=($!)
    sleep 2  # stagger startup rather than launching all instances in the same instant
done

echo ""
echo "All instances launched. Shared reference point for mavlink_shim.py's"
echo "--common-origin (same for every shim, regardless of that shim's own"
echo "vehicle's distinct home above): ${BASE_LAT},${BASE_LON},${BASE_ALT}"
echo ""

trap 'echo "Stopping all SITL instances..."; kill "${pids[@]}" 2>/dev/null' INT TERM
wait
