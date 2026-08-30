#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "fuzzylib/blocks/block.hpp"
#include "fuzzylib/hdc/hypervector.hpp"

namespace fuzzylib::blocks {

// A symbolic "regime recognizer": encodes several named scalar signals into
// a single hypervector (binding each value to a role vector for its input
// name, then bundling across inputs), matches it against a library of
// labeled prototype hypervectors, and emits the numeric value associated
// with the closest prototype plus a confidence score. Useful for turning a
// continuous local situation (e.g. neighbor density in a swarm) into a
// discrete mode that gates a Blend's weights.
class HDCPrototypeBlock final : public IBlock {
public:
    explicit HDCPrototypeBlock(std::shared_ptr<hdc::HDCSpace> space) : space_(std::move(space)) {}

    HDCPrototypeBlock& addInput(const std::string& key, double min, double max, std::size_t levels) {
        encoders_.emplace(key, hdc::ScalarEncoder(*space_, min, max, levels, seedFor(key)));
        roles_.emplace(key, space_->random());
        inputOrder_.push_back(key);
        return *this;
    }

    HDCPrototypeBlock& addPrototype(const std::string& label,
                                      const std::unordered_map<std::string, double>& exampleInputs,
                                      double outputValue) {
        memory_.add(label, encodeInputs(exampleInputs));
        outputValues_[label] = outputValue;
        return *this;
    }

    Signals evaluate(const Signals& inputs) override {
        const hdc::Hypervector query = encodeInputs(inputs);
        const auto [label, similarity] = memory_.nearest(query);
        Signals out;
        out["value"] = outputValues_.count(label) ? outputValues_.at(label) : 0.0;
        out["confidence"] = similarity;
        return out;
    }

    std::string name() const override { return "hdc_prototype"; }

private:
    static unsigned seedFor(const std::string& key) {
        return static_cast<unsigned>(std::hash<std::string>{}(key) ^ 0x9e3779b9u);
    }

    hdc::Hypervector encodeInputs(const std::unordered_map<std::string, double>& inputs) const {
        std::vector<hdc::Hypervector> parts;
        parts.reserve(inputOrder_.size());
        for (const auto& key : inputOrder_) {
            const double value = inputs.count(key) ? inputs.at(key) : 0.0;
            const hdc::Hypervector encoded = encoders_.at(key).encode(value);
            parts.push_back(space_->bind(roles_.at(key), encoded));
        }
        return space_->bundle(parts);
    }

    std::shared_ptr<hdc::HDCSpace> space_;
    std::vector<std::string> inputOrder_;
    std::unordered_map<std::string, hdc::ScalarEncoder> encoders_;
    std::unordered_map<std::string, hdc::Hypervector> roles_;
    hdc::ItemMemory memory_;
    std::unordered_map<std::string, double> outputValues_;
};

}  // namespace fuzzylib::blocks
