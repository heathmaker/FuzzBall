# FuzzyRedo

A modular C++17 fuzzy logic library that scales from a basic Mamdani fuzzy
inference system (FIS) up to complex Takagi-Sugeno-Kang (TSK/Sugeno)
systems, and blends cleanly with other AI/control techniques — PID,
genetic algorithms, neural networks, and hyperdimensional computing (HDC)
— behind one common interface.

## Why it's structured this way

Everything in the library — a fuzzy inference system, a PID loop, a neural
net, an HDC classifier — implements the same tiny interface:

```cpp
class IBlock {
public:
    virtual Signals evaluate(const Signals& inputs) = 0;
    virtual void reset() {}
};
```

where `Signals` is just `unordered_map<string, double>`. Because every
technique looks the same from the outside, they compose the same way:

- **`Pipeline`** runs blocks in sequence, threading a growing signal map
  through them (each stage sees the original inputs plus everything
  produced so far).
- **`Blend`** fans a set of blocks out over the same inputs and combines
  their outputs as a weighted average — the weight for each member can be
  a fixed constant, or read live from a signal produced earlier in the
  pipeline (e.g. a small fuzzy "scheduler" block deciding how much to
  trust an aggressive vs. a gentle PID controller).

This is the mechanism behind "blending fuzzy and other AI methods": you
are never fighting a bespoke fuzzy-only API to bolt on a PID loop or a
neural net — you write one, wrap it as an `IBlock` (a wrapper already
exists for PID, NN, and HDC), and drop it into a `Pipeline`/`Blend`.

## Layout

```
include/fuzzylib/
  core/         membership functions, fuzzy sets, linguistic variables,
                t-norms/t-conorms, the antecedent expression tree, rules
  inference/    MamdaniEngine (classical FIS) and SugenoEngine (TSK)
  blocks/       IBlock, Pipeline, Blend, and wrappers (FIS/PID/NN/HDC)
  control/      PIDController
  ga/           a real-coded genetic algorithm
  nn/           a small feedforward network with backprop
  hdc/          hyperdimensional computing primitives
src/            .cpp implementations for the above
examples/       tipper, water_dam, inverted_pendulum, car_platooning,
                quadcopter, drone_swarm
tests/          a small dependency-free test harness + unit tests
```

## Building

```sh
cmake -S . -B build
cmake --build build -j
ctest --test-dir build          # run the unit tests
./build/examples/tipper         # run any example
```

CMake options (all default `ON` except the warnings one):
`FUZZYLIB_BUILD_EXAMPLES`, `FUZZYLIB_BUILD_TESTS`,
`FUZZYLIB_WARNINGS_AS_ERRORS`.

## The fuzzy core

- **Membership functions** (`core/membership.hpp`): triangular,
  trapezoidal, Gaussian, generalized bell, sigmoid, S/Z-shape, singleton,
  constant, and a `Custom` wrapper for anything else.
- **Linguistic variables** (`core/variable.hpp`) hold a universe of
  discourse and a set of named terms.
- **Antecedents** (`core/rule.hpp`) are a small expression tree —
  `antecedent::is("service", "good")`, combined with `&`, `|`, `~`, and
  optional hedges (`Hedge::Very`, `Hedge::Somewhat`, ...). AND/OR are
  pluggable via `TNorm` (Zadeh min/max by default, or algebraic
  product/probabilistic-sum).
- **Rules** attach either a `MamdaniConsequent` ("output IS term") or a
  `SugenoConsequent` (`output = constant + Σ coeff_i * input_i` — the "TSK"
  in Takagi-Sugeno-Kang; an empty `coefficients` map makes it zero-order).

## Mamdani vs. Sugeno

Both engines share the same fuzzification/antecedent machinery; they
differ only in how a rule's contribution becomes a crisp number:

- **`MamdaniEngine`**: clips/scales each firing rule's output fuzzy set,
  aggregates them (max, by default) into one membership curve per output
  variable, then defuzzifies (`Centroid`, `Bisector`, `MeanOfMax`,
  `SmallestOfMax`, `LargestOfMax`). Intuitive and interpretable — this is
  what the `tipper` and `water_dam` examples use.
- **`SugenoEngine`**: every rule computes its own crisp value directly
  from a linear function of the inputs; the output is the
  firing-strength-weighted average across rules. No output universe to
  discretize, so it scales to many rules/inputs and composes well with
  gradient- or search-based tuning — this is what `inverted_pendulum` and
  `car_platooning` use (both build a "complex" many-rule TSK controller
  by generating rules programmatically rather than authoring them by
  hand, which is what scaling a FIS up really looks like in practice).

## Examples

| Example | What it shows |
|---|---|
| `tipper` | The classic Mamdani tipping problem — the "hello world" of fuzzy logic. |
| `water_dam` | Mamdani gate control holding a reservoir in a safe band through a scripted storm, on a simple tank simulation. |
| `inverted_pendulum` | A 25-rule TSK controller stabilizing a nonlinear cart-pole, then a **GA tuning the FIS's force gain** against a closed-loop cost — fuzzy + genetic algorithm. |
| `car_platooning` | A 9-rule gain-scheduled TSK adaptive-cruise-control law, reused unmodified across a 5-vehicle chain, absorbing a hard-brake disturbance. |
| `quadcopter` | A full **6DoF** rigid-body flight sim (quaternion attitude, 4-motor thrust mixing) flown by a cascaded position-PID -> fuzzy-scheduled-PID-attitude -> motor-mixer controller — the canonical fuzzy-PID hybrid, now driving a real 3D vehicle instead of one decoupled axis. |
| `drone_swarm` | 3D boids-style flocking past static obstacles: each drone classifies local crowding via an **HDC prototype block** to gain-schedule separation/cohesion, and separately runs a **Mamdani FIS** on obstacle clearance to gain-schedule avoidance — fuzzy logic and HDC doing the same kind of local-context scheduling side by side. |

Each example is a single, runnable, verbose-output `.cpp` file meant to be
read top to bottom as documentation of one usage pattern.

## Flying a real ArduPilot vehicle in SITL (and Mission Planner)

`examples/ardupilot_bridge/` links a fuzzylib controller to a real
ArduPilot vehicle running in Software In The Loop, so it can be watched
in Mission Planner (or QGroundControl, or plain MAVProxy) like any other
MAVLink vehicle. It's a **guidance-layer** link, not a full replacement of
ArduPilot's flight stack: ArduPilot's own EKF, attitude controller, and
motor mixing keep flying the vehicle exactly as they always do — our
controller just streams GUIDED-mode velocity/yaw setpoints, the same way
a companion computer (a Pi running ROS2/MAVROS, say) commands a real
ArduPilot vehicle in the field.

**Why two processes.** MAVLink is a binary protocol with per-message CRCs
that are impractical to get right by hand (and pointless to — this is a
solved, "use the reference implementation" problem, not a fuzzy-logic
one). So a small Python script, `tools/ardupilot_bridge/mavlink_shim.py`,
owns all MAVLink traffic via [pymavlink](https://github.com/ArduPilot/pymavlink)
(the same library ArduPilot's own test suite and MAVProxy are built on):
it does the one-time handshake (wait for heartbeat, switch to GUIDED, arm,
take off), then in a loop relays vehicle telemetry to our C++ controller
and its velocity/yaw replies back to ArduPilot as real
`SET_POSITION_TARGET_LOCAL_NED` messages — over a local JSON-over-UDP
protocol documented at the top of both files. The C++ side,
`examples/ardupilot_bridge/ardupilot_bridge.cpp`, is pure fuzzylib: a
3-rule zero-order Sugeno FIS maps distance-to-waypoint to an approach
speed that tapers smoothly on arrival (instead of a constant cruise speed
that would overshoot), flying a 5-waypoint square circuit.
`tools/ardupilot_bridge/test_protocol.py` exercises the controller's
guidance logic and its half of the wire protocol without needing a real
SITL instance at all, by playing the shim's role itself.

This has been verified end to end against a real ArduCopter build (tagged
stable release `Copter-4.6.3` — build against a stable tag, not `master`,
which at the time of writing rejects `MAV_CMD_NAV_TAKEOFF` outright): the
vehicle arms, climbs, and flies all four waypoints with the commanded
speed visibly tapering from ~6 m/s cruise down to ~0.4 m/s on each
approach, exactly matching the guidance FIS.

**Running it:**

```sh
# 1. Install ArduPilot SITL from a stable release tag (one-time; see
#    ArduPilot's own build docs) and pymavlink: pip install pymavlink

# 2. Start SITL (from an ardupilot checkout):
Tools/autotest/sim_vehicle.py -v ArduCopter --console --map

# 3. Start the fuzzylib guidance controller:
./build/examples/ardupilot_bridge

# 4. Start the MAVLink shim:
python3 tools/ardupilot_bridge/mavlink_shim.py
```

Mission Planner can connect to the same SITL instance (default
`udp:127.0.0.1:14550`) at any point in that sequence — it only ever talks
MAVLink to ArduPilot, never to the shim or the controller, so from its
point of view this is just a normal vehicle flying a normal GUIDED-mode
mission.

**A freshly-wiped SITL vehicle won't arm out of the box.** `mavlink_shim.py`
sets four parameters right after connecting — `FRAME_CLASS`/`FRAME_TYPE`
(normally injected by `sim_vehicle.py`'s `-f`/`--frame` option, which
launching the raw `arducopter` binary directly, e.g. to skip its
MAVProxy/wx dependency, bypasses) and `ARMING_CHECK`/`DISARM_DELAY`
(disabled so a simulated vehicle with no real accelerometer calibration
or GPS-lock history isn't blocked by, or kicked off the ground while
waiting on, safety checks that exist for real hardware). **Never disable
those on a real vehicle.** It then retries the arm/takeoff sequence for
several minutes if needed: ArduPilot's EKF/AHRS converge on their own
schedule after boot, not a fixed delay after the first heartbeat, so a
single-shot attempt can time out even though the vehicle would have
accepted the same command moments later.

**Safety note:** unlike PX4's offboard mode, ArduPilot's GUIDED mode does
not automatically abort if the setpoint stream stops — it keeps flying
the last commanded velocity. That's an acceptable simplification for
SITL (which is what this bridge is built and tested against) but would
need a watchdog (fail to RTL/hover on stale telemetry) before pointing it
at a real vehicle.

## Extending it

- **New membership function**: subclass `MembershipFunction`
  (`core/membership.hpp`), implement `operator()` and `clone()`.
- **New inference behavior**: implement `IBlock` directly — you don't need
  to go through `MamdaniEngine`/`SugenoEngine` at all if you want, say, a
  hand-rolled fuzzy classifier.
- **New way to combine techniques**: `Blend`'s weights are just signal
  names, so any upstream block (fuzzy, HDC, hard-coded heuristic) can
  drive how much another block's output counts.
