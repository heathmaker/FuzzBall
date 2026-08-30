#pragma once

#include <memory>
#include <string>
#include <vector>

#include "fuzzylib/blocks/block.hpp"
#include "fuzzylib/nn/feedforward.hpp"

namespace fuzzylib::blocks {

// Wraps a FeedForwardNetwork as an IBlock: named input signals are gathered
// in `inputKeys` order into the network's input vector, and the network's
// output vector is scattered back out under `outputKeys`.
class NNBlock final : public IBlock {
public:
    NNBlock(std::shared_ptr<nn::FeedForwardNetwork> network, std::vector<std::string> inputKeys,
             std::vector<std::string> outputKeys)
        : network_(std::move(network)), inputKeys_(std::move(inputKeys)), outputKeys_(std::move(outputKeys)) {}

    Signals evaluate(const Signals& inputs) override {
        std::vector<double> in;
        in.reserve(inputKeys_.size());
        for (const auto& key : inputKeys_) in.push_back(inputs.at(key));

        const std::vector<double> out = network_->forward(in);
        Signals result;
        for (std::size_t i = 0; i < outputKeys_.size() && i < out.size(); ++i) {
            result[outputKeys_[i]] = out[i];
        }
        return result;
    }

    std::string name() const override { return "nn"; }

    nn::FeedForwardNetwork& network() { return *network_; }

private:
    std::shared_ptr<nn::FeedForwardNetwork> network_;
    std::vector<std::string> inputKeys_;
    std::vector<std::string> outputKeys_;
};

}  // namespace fuzzylib::blocks
