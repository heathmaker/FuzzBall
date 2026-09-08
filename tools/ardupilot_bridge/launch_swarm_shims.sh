#!/bin/bash
# Launches one mavlink_shim.py per ArduCopter SITL instance started by
# launch_swarm_sitl.sh, each paired with its own UDP port on
# ardupilot_swarm_bridge (start that controller first). Every shim is
# given the *same* --common-origin (launch_swarm_sitl.sh's base
# location) so each converts its own vehicle's independent GPS fix into
# one shared local frame -- required because each vehicle's own
# LOCAL_POSITION_NED is relative to its own EKF origin and isn't
# otherwise comparable to another vehicle's. Stop all shims with Ctrl+C.
#
# Usage: NUM_AGENTS=3 bash launch_swarm_shims.sh
# (SWARM_BASE_LAT/LON/ALT must match whatever launch_swarm_sitl.sh used.)

set -e
NUM_AGENTS="${NUM_AGENTS:-3}"
BASE_LAT="${SWARM_BASE_LAT:-40.071374}"
BASE_LON="${SWARM_BASE_LON:--105.229195}"
BASE_ALT="${SWARM_BASE_ALT:-1600}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

pids=()
for i in $(seq 0 $((NUM_AGENTS - 1))); do
    mavlink_port=$((14550 + i * 10))
    controller_port=$((6001 + i))
    echo "Starting shim $i: SITL udp:127.0.0.1:$mavlink_port <-> controller port $controller_port"
    python3 "$SCRIPT_DIR/mavlink_shim.py" \
        --connect "udp:127.0.0.1:$mavlink_port" \
        --controller-port "$controller_port" \
        --common-origin "${BASE_LAT},${BASE_LON},${BASE_ALT}" \
        --takeoff-alt 3.0 &
    pids+=($!)
done

trap 'echo "Stopping all shims..."; kill "${pids[@]}" 2>/dev/null' INT TERM
wait
