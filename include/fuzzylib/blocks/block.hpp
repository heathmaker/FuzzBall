#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace fuzzylib::blocks {

// A named bag of scalar signals flowing between blocks (crisp values only —
// fuzzification happens inside whichever block needs it).
using Signals = std::unordered_map<std::string, double>;

// Common interface for anything that consumes signals and produces signals:
// a fuzzy inference system, a PID loop, a neural net, a GA-tuned controller,
// an HDC classifier, or a composite of any of the above. This is what makes
// the library "modular": every technique looks the same from the outside.
class IBlock {
public:
    virtual ~IBlock() = default;
    virtual Signals evaluate(const Signals& inputs) = 0;
    // Clears any internal state (integrators, filters, recurrent memory, ...).
    virtual void reset() {}
    virtual std::string name() const { return "block"; }
};

using BlockPtr = std::shared_ptr<IBlock>;

// Runs a sequence of named blocks, threading a growing signal map through
// them: each stage sees the original inputs plus every output produced by
// earlier stages, so later blocks can consume earlier blocks' results.
class Pipeline final : public IBlock {
public:
    Pipeline& add(std::string stageName, BlockPtr block) {
        stages_.push_back({std::move(stageName), std::move(block)});
        return *this;
    }

    Signals evaluate(const Signals& inputs) override {
        Signals state = inputs;
        for (auto& stage : stages_) {
            Signals out = stage.block->evaluate(state);
            for (auto& [k, v] : out) state[k] = v;
        }
        return state;
    }

    void reset() override {
        for (auto& stage : stages_) stage.block->reset();
    }

    std::string name() const override { return "pipeline"; }

private:
    struct Stage {
        std::string name;
        BlockPtr block;
    };
    std::vector<Stage> stages_;
};

// One contributor to a Blend: which block, which of its outputs to use, and
// how heavily to weight it. If weightKey is set, the weight is read from the
// signals passed into the blend on every cycle (e.g. produced upstream by a
// fuzzy gain-scheduler block in the same Pipeline) instead of being fixed —
// this is the mechanism for fuzzy-blended hybrid controllers.
struct BlendMember {
    BlockPtr block;
    std::string outputKey;
    std::string weightKey;
    double staticWeight = 1.0;
};

// Combines several blocks' outputs into a single weighted-average signal.
// All members are evaluated against the same inputs (fan-out, not a chain).
class Blend final : public IBlock {
public:
    Blend(std::vector<BlendMember> members, std::string outputName)
        : members_(std::move(members)), outputName_(std::move(outputName)) {}

    Signals evaluate(const Signals& inputs) override {
        double weightedSum = 0.0;
        double weightSum = 0.0;
        for (const auto& member : members_) {
            const Signals out = member.block->evaluate(inputs);
            const double value = out.at(member.outputKey);
            double weight = member.staticWeight;
            if (!member.weightKey.empty()) {
                auto it = inputs.find(member.weightKey);
                if (it != inputs.end()) weight = it->second;
            }
            weightedSum += weight * value;
            weightSum += weight;
        }
        Signals result;
        result[outputName_] = weightSum > 0.0 ? weightedSum / weightSum : 0.0;
        return result;
    }

    void reset() override {
        for (auto& member : members_) member.block->reset();
    }

    std::string name() const override { return "blend"; }

private:
    std::vector<BlendMember> members_;
    std::string outputName_;
};

}  // namespace fuzzylib::blocks
