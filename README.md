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
| `quadcopter` | A fuzzy Mamdani *scheduler* blending a gentle and an aggressive PID controller per axis — the canonical fuzzy-PID hybrid — stabilizing roll/pitch/yaw. |
| `drone_swarm` | Boids-style flocking where each drone classifies local crowding via an **HDC prototype block** and uses it to gain-schedule separation/cohesion — fuzzy-style scheduling driven by hyperdimensional computing instead of a FIS. |

Each example is a single, runnable, verbose-output `.cpp` file meant to be
read top to bottom as documentation of one usage pattern.

## Extending it

- **New membership function**: subclass `MembershipFunction`
  (`core/membership.hpp`), implement `operator()` and `clone()`.
- **New inference behavior**: implement `IBlock` directly — you don't need
  to go through `MamdaniEngine`/`SugenoEngine` at all if you want, say, a
  hand-rolled fuzzy classifier.
- **New way to combine techniques**: `Blend`'s weights are just signal
  names, so any upstream block (fuzzy, HDC, hard-coded heuristic) can
  drive how much another block's output counts.
