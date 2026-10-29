# FuzzBall

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
                quadcopter, drone_swarm, thesis_platoon (+ the bridges)
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
| `sumo_acc` | The same `car_platooning` ACC law, this time tracking a real **SUMO**-simulated human driver over 10 miles of highway (see below) instead of a scripted leader profile. |
| `thesis_platoon` | A C++ replication of the **Master's thesis** *Optimizing Platoon Time Gap Following on the Highway using Genetic Fuzzy Systems*: a four-car platoon holding a 1 s time gap behind a human driver in SUMO, with hand-picked (FIS) and **GA-tuned (GFS)** Mamdani controllers for car-following, lane changing and post-lane-change leading (see below). |

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

# 5. Point Mission Planner at the same SITL instance, at any point in
#    this sequence: connection-type dropdown (top right) -> UDP -> Connect
#    -> host 127.0.0.1, port 14550 (sim_vehicle.py's default MAVProxy
#    output for exactly this purpose). If Mission Planner runs on a
#    different machine/VM than SITL, use that machine's own address
#    instead of 127.0.0.1, and make sure port 14550 is reachable from it.
```

Mission Planner only ever talks MAVLink to ArduPilot — never to the shim
or the controller — so from its point of view this is just a normal
vehicle flying a normal GUIDED-mode mission; it can attach at any point in
the sequence above, before or after the bridge/shim are running.

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

## Flying a real multi-vehicle ArduPilot swarm in SITL

`examples/ardupilot_bridge/ardupilot_swarm_bridge.cpp` runs the same
boids + HDC-gain-scheduled flocking behavior as `examples/drone_swarm`
(separation/cohesion/alignment/goal-seeking, with an `HDCPrototypeBlock`
scheduling the separation/cohesion gains off local crowding) — but over
several real, independent ArduCopter SITL instances instead of that
example's own internal rigid-body simulation. One controller process
manages N agents at once: it opens one UDP port per agent (paired with its
own `mavlink_shim.py` and SITL instance, same wire protocol as the
single-vehicle bridge), caches every agent's latest reported position and
velocity, and on each agent's state update recomputes the full flock's
behavior using whatever neighbor data is freshest — then replies with just
that agent's velocity setpoint.

**Coordinate frames are the hard part of "multi-vehicle."** Every vehicle's
`LOCAL_POSITION_NED` is relative to its own EKF origin, so raw positions
from different vehicles are never directly comparable — true in SITL, and,
more importantly, true of any two physically separate real vehicles (which
obviously can't share a GPS origin the way co-located SITL instances
technically could). `mavlink_shim.py` fixes this with a `--common-origin
lat,lon,alt_m` option: every shim projects its own vehicle's absolute GPS
fix (`GLOBAL_POSITION_INT`) onto the same shared flat-Earth reference point
using an equirectangular projection, so all agents' reported positions land
in one common local frame regardless of how far apart their actual homes
are. `launch_swarm_sitl.sh` gives each SITL instance a genuinely distinct
home (a small staged launch line, the way real vehicles would actually be
laid out) and prints the shared reference point for `launch_swarm_shims.sh`
to pass to every shim.

**Running it (3 agents by default):**

```sh
# 1. From an ArduPilot checkout, launch the SITL swarm (each instance N's
#    MAVLink lands on udp:127.0.0.1:14550+10N, ArduPilot's usual per-
#    instance offset):
NUM_AGENTS=3 bash /path/to/fuzzylib/tools/ardupilot_bridge/launch_swarm_sitl.sh

# 2. Start the swarm guidance controller (one process, N UDP ports):
./build/examples/ardupilot_swarm_bridge

# 3. Start the shims (one per SITL instance, sharing the base location
#    launch_swarm_sitl.sh printed):
NUM_AGENTS=3 bash tools/ardupilot_bridge/launch_swarm_shims.sh

# 4. Point Mission Planner (or QGroundControl/MAVProxy) at any instance's
#    MAVLink UDP port, same as the single-vehicle bridge.
```

`tools/ardupilot_bridge/test_swarm_protocol.py` exercises the controller's
per-agent UDP protocol and flocking logic without needing SITL at all, by
playing all N shims' role itself: goal-seeking when isolated, separation
when crowded, per-port isolation, and braking on arrival.

This has been verified end to end against 3 real ArduCopter SITL instances:
all three arm, climb, and fly under the shared flocking controller,
converging on distinct slots around a common goal and holding position
there. Two bugs surfaced only once vehicles actually reached the goal
(never observed in `drone_swarm`'s pure simulation, whose goal was far
enough away that it was never actually reached in the simulated window):

- **Missing velocity damping.** `velocityCmd = measured_velocity +
  accel*dt` carries speed forward with nothing to bleed it off as
  goal-seeking acceleration shrinks near arrival (alignment only levels
  differences *between* agents' velocities — it doesn't slow the flock
  down as a whole). Without damping, an agent flew straight through the
  goal at cruise speed and kept going.
- **A single shared goal point is unstable once reached.** With every
  agent seeking the exact same point, separation (pushing agents apart)
  and goal-seeking (pulling all of them together) fight indefinitely once
  the flock converges — confirmed live as a persistent, only slowly-decaying
  orbit around the goal instead of settling. Fixed by giving each agent its
  own slot on a small ring around the shared goal, the same way a real
  multi-vehicle rendezvous would need distinct slots (vehicles can't
  occupy the same point either). The velocity-damping constant also needed
  to sit above the critical-damping point for the resulting goal-seeking
  spring (`accel ≈ kGoalGain * (slot - pos)` near arrival gives a natural
  frequency of `sqrt(kGoalGain)`); underdamped, the flock still settled
  into a wide, slowly-decaying orbit around its formation ring rather than
  holding position.

The same safety note as the single-vehicle bridge applies: ArduPilot's
GUIDED mode keeps flying the last commanded velocity if the setpoint
stream stops, which is fine for SITL but would need a watchdog before
pointing this at real hardware.

## SUMO adaptive cruise control

`examples/sumo_acc/sumo_acc.cpp` hooks the exact same 9-rule gain-scheduled
TSK ACC law as `car_platooning` up to a real [SUMO](https://eclipse.dev/sumo/)
traffic simulation, over [libtraci](https://sumo.dlr.de/docs/libtraci.html)
(SUMO's own native C++ TraCI client library) instead of `car_platooning`'s
internal point-mass physics. The scenario is a straight, one-lane, 10-mile
highway (`tools/sumo_acc/network.edg.xml`) with a 65 mph limit that drops to
45 mph for a 2-mile stretch in the middle. The lead vehicle is SUMO's own
default **Krauss car-following model** — the standard human-driver model,
complete with its usual driver imperfection (`sigma`) and reaction time
(`tau`) — left otherwise unmodified; it slows for and re-accelerates out of
the 45 mph zone entirely under SUMO's own logic, so the ACC gets a real
disturbance to track without any scripted velocity profile on our side (the
role `car_platooning`'s hard-brake profile plays there). Like the ArduPilot
bridges, this program owns no physics itself: it only supplies the following
vehicle's per-step speed command, the same guidance-layer role
`ardupilot_bridge` plays for a real ArduPilot vehicle.

The ACC's desired following distance is a dynamic, speed-based time gap
rather than a fixed number of meters: a 1-second gap at and above a 25 mph
floor speed (e.g. ~27.5 m at the 65 mph cruise speed, ~19 m in the 45 mph
zone), held at its 25-mph value below that floor rather than letting it
keep shrinking toward 0 while crawling or stopped — the shape a real ACC's
distance setting has, and `gapError` is exactly actual-gap-minus-this.

**Setup (one-time):**

```sh
pip install eclipse-sumo traci sumolib
export SUMO_HOME="$(python3 -c 'import sumo, os; print(os.path.dirname(sumo.__file__))')"
```

`SUMO_HOME` is the same environment variable all of SUMO's own tooling uses;
CMake looks for `$SUMO_HOME/include/libsumo` and `$SUMO_HOME/lib64/libtracicpp.*`
to build `sumo_acc` — if it isn't set (or SUMO isn't installed), CMake prints
`SUMO libtraci not found -- skipping sumo_acc example` and every other target
still builds normally.

**Running it:**

```sh
# 1. Configure with SUMO_HOME set (from a fresh build/ if you configured
#    before installing SUMO), then build as usual.
cmake -S . -B build && cmake --build build

# 2. Compile the network (only needed once, or after editing the .nod.xml/
#    .edg.xml source files):
bash tools/sumo_acc/build_network.sh

# 3. Run it (headless; pass --gui for sumo-gui instead):
./build/examples/sumo_acc tools/sumo_acc/sumo.sumocfg

# 4. Optional: record a chase-cam video via sumo-gui's screenshot API
#    (implies --gui; needs an actual X display, e.g. a real desktop or
#    Xvfb). Off by default -- a full run captures on the order of 800
#    PNGs, so only pass this when you actually want a recording:
./build/examples/sumo_acc tools/sumo_acc/sumo.sumocfg --record /tmp/sumo_acc_frames
# then, once it finishes, encode the frames (the run prints this exact command):
ffmpeg -framerate 10 -i /tmp/sumo_acc_frames/frame_%06d.png \
       -c:v libx264 -pix_fmt yuv420p /tmp/sumo_acc_frames/sumo_acc_demo.mp4
```

**Known caveat:** `--gui`/`--record` drive `sumo-gui` over a real X display.
In at least one headless/Xvfb sandbox, a full run occasionally hung
indefinitely partway through — reproducibly, and identically with plain
`--gui` and no recording involved at all, so it's an interaction between
`sumo-gui` and `libtraci` under that specific setup rather than a bug in
this file. The default headless mode (no `--gui`, no `--record`) uses a
completely different SUMO code path and has never shown this. A real
desktop X session may not hit it at all; if a recording run does seem
stuck, that's the known issue, not a hang worth debugging in your own code.

The ego vehicle departs from a standstill 50 m behind the leader (which
starts already at speed), so the largest gap error in the printed log is
this initial catch-up transient, not a tracking failure — watch the numbers
converge to a near-zero, steady gap error both before and after the 45 mph
zone. The leader's realized speed settles a little under the posted limits
(SUMO samples each simulated driver its own fixed `speedFactor`, e.g. ~61.5
mph against a 65 mph limit for a given run/seed) — the ACC correctly tracks
whatever the human actually does, not the road's nominal speed limit, which
is exactly what a real ACC targets.

## Master's thesis replication: genetic fuzzy platooning

`examples/thesis_platoon/` is a port of the Python code in
[`HeathPalmer/master_thesis`](https://github.com/HeathPalmer/master_thesis)
(`lane_change_controller` branch, `test/FIS`) to C++: the thesis *Optimizing
Platoon Time Gap Following on the Highway using Genetic Fuzzy Systems*
(Palmer, Sathyan, Cohen; NAFIPS 2024). Four connected vehicles follow a
human-driven (Krauss) lead on a 40-mile SUMO highway. After a 300 s Krauss
warm-up the followers switch to the model under test and try to hold a 1 s
time gap to the vehicle ahead:

| Model | What drives the four followers |
|---|---|
| `Krauss` | SUMO's human-driver model (baseline) |
| `CACC` | SUMO's Cooperative Adaptive Cruise Control model (baseline) |
| `FIS` | three hand-picked Mamdani systems: gap error + its rate → acceleration (7 rules); average time loss + its rate → change lanes (2 rules); and, once the platoon has changed lanes and no longer follows the human, platoon gap error + velocity error → the leader's acceleration (3 rules) |
| `GFS` | the same rule bases with membership functions tuned by a genetic algorithm — one GA per controller, 66 / 24 / 27 genes |

The four scenarios are the thesis' own: **1** single lane, cruising at 70 mph;
**2** single lane, the lead slows to ~55 mph at about 9 minutes; **3** two
lanes, cruising (the platoon may overtake the slow lead); **4** two lanes with
the slowdown. The scenario files are copied verbatim from the thesis
(`tools/thesis_platoon/highway_2*`; `highway_2_mod` is the single-lane road).

**What was converted from Python**

| Thesis (Python) | Here (C++) |
|---|---|
| scikit-fuzzy `ControlSystem` / `trimf` | `fuzzylib::MamdaniEngine` (`thesis_fis.cpp`) |
| EasyGA `GA` + the three `test_easy_ga_*.py` trainers | `easy_ga.cpp` + `thesis_train.cpp` |
| `traci` | `libtraci` (`scenario.cpp`) |
| `test_sumo_script.py` `run()` | `scenario.cpp` — all four models, all four scenarios |
| pandas + `xml2csv` + matplotlib | CSV and SVG written by `report.cpp` |

The controllers reproduce scikit-fuzzy to floating-point precision (the unit
tests pin them against values the original Python produces). To get there
`MamdaniEngine` gained two general options, both off by default:
`setRefineAtCuts(true)` (scikit-fuzzy inserts the points where each output
term crosses its clip level into the output grid, so clipped triangles keep
their corners on a coarse grid) and `DefuzzMethod::CentroidExact` (the exact
centroid of the piecewise-linear aggregate), plus `evaluateStrict()` which
throws when no rule fires instead of defuzzifying to the middle of the
universe — scikit-fuzzy raises there too, and the thesis' GA scored such
candidates 100000.

**Running it**

```sh
# The thesis used SUMO 1.16.0; that version reproduces its numbers exactly.
pip install eclipse-sumo==1.16.0
export SUMO_HOME="$(python3 -c 'import sumo, os; print(os.path.dirname(sumo.__file__))')"
export PATH="$SUMO_HOME/bin:$PATH"

# That wheel is built with libstdc++'s old string ABI (SUMO 1.27's is not);
# if linking fails with undefined libtraci:: symbols, add this flag:
cmake -S . -B build -DCMAKE_CXX_FLAGS=-D_GLIBCXX_USE_CXX11_ABI=0
cmake --build build

# One scenario, all four models (about 40 s); writes CSV + SVG plots and summary.csv
./build/examples/thesis_run --scenario 3 --out results/scenario3
./build/examples/thesis_run --scenario 4 --model gfs --gui     # watch one in sumo-gui

# Compare against the paper's published numbers (after running scenarios 1-4)
python3 tools/thesis_platoon/compare_to_paper.py results
```

Each run writes, per model: `<Model>_Gap_error.csv` / `_Gap_error_rate.csv`
(the thesis' spreadsheets), `<Model>_trace.csv` (position, speed and
acceleration of every vehicle each second) and SVG plots of gap error, time
gap, position, velocity, acceleration, jerk, time-to-collision (with the 3 s
safety threshold) and the lane-change decision.

**Does it reproduce the thesis?** Yes — 46 of the 48 published numbers (total
gap error, total time lost and minimum TTC for four models in four
scenarios) match, to the paper's own rounding. Table 1 of the paper
(scenario 3):

| Model | Total gap error (s) | Total time lost (s) | Min TTC (s) |
|---|---|---|---|
| Krauss | 13088.77 | 249798.79 | 51.84 |
| CACC | 4969.88 | 170297.73 | 9.38 |
| FIS | 2215.60 | 149507.65 (paper: 143580.34) | 26.01 |
| GFS | 1943.87 | 118627.06 | 26.00 |

The two that differ are both hand-picked-FIS runs: the FIS time lost above,
and the scenario 1 FIS minimum TTC (26.68 s here, 26.82 s in the slides). In
both, the other published metric for the same run matches exactly, and in the
slides the scenario 1 and 2 FIS TTC differ even though the two runs are
identical until the lead slows down at ~9 minutes, so those two figures most
likely came from a slightly different parameter set than the final
`test_sumo_script.py` the port follows.

**Training the GFS again**

```sh
./build/examples/thesis_train --stage lon1 --generations 5 --out results/train_lon1
```

`--stage` is `lon1` (66 genes), `lat` (24) or `lon2` (27); the controllers
not being trained stay hand-picked, as in the thesis, and a candidate's
fitness is the platoon's summed absolute time-gap error over a two-lane
cruising run. The thesis' settings (10 chromosomes, 500 generations) are the
defaults, and since each fitness evaluation is a full SUMO run that takes
hours; the GFS chromosomes the thesis arrived at are already in
`thesis_params.cpp`, so retraining is optional. The best chromosome is
written to `best_<stage>.txt` both in the thesis' format and as C++ triples.

**Kept as they were in the Python** (so the numbers stay comparable): the
TTC calculation takes the speed difference against the vehicle *ahead of* each
car rather than the follower; the fitness window slices the series by list
index rather than by simulation step; a comparison of lane-id strings with the
integer 1 is always true, which makes the "platoon not yet in the left lane"
branch nudge vehicles 2 and 4 down and vehicle 3 up by 0.5 m/s every step; and
vehicle 1 records no gap error while it leads in the left lane, so the
per-vehicle series differ in length (the thesis' CSVs are cut to the shortest
one). In the GA, EasyGA's default gene mutation replaces a gene with a random
integer 1–10 — the integer genes in the thesis' trained chromosomes — and
its survivor-selection override has an extra path that keeps a failed
chromosome after replacing it. These are worth knowing about if you plan to
build on the work.

**Differences**: lane indices are parsed from the end of the lane id
(`int(lane_id[6])` in the Python breaks on junction-internal lanes); the
trainers use the same scenario runner as the evaluation script and stop
simulating at 2000 s (nothing after that is scored) rather than carrying their
own copy of `run()`; and the lat trainer scores total gap error like the other
two, as its code does, although the thesis text says total time lost
(`--objective time-lost` selects that).

## Extending it

- **New membership function**: subclass `MembershipFunction`
  (`core/membership.hpp`), implement `operator()` and `clone()`.
- **New inference behavior**: implement `IBlock` directly — you don't need
  to go through `MamdaniEngine`/`SugenoEngine` at all if you want, say, a
  hand-rolled fuzzy classifier.
- **New way to combine techniques**: `Blend`'s weights are just signal
  names, so any upstream block (fuzzy, HDC, hard-coded heuristic) can
  drive how much another block's output counts.
