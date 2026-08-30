#pragma once

#include <cstddef>
#include <vector>

namespace fuzzylib::nn {

enum class Activation { Linear, ReLU, Tanh, Sigmoid };

// A small fully-connected feedforward network with backprop training, meant
// to be blended with fuzzy/PID controllers (e.g. as a learned residual
// compensator) rather than to compete with a full deep-learning framework.
class FeedForwardNetwork {
public:
    // layerSizes.size() must equal activations.size() + 1, e.g. {2,8,1} with
    // activations {Tanh, Linear} describes a 2-input, 8-hidden-unit, 1-output net.
    FeedForwardNetwork(std::vector<std::size_t> layerSizes, std::vector<Activation> activations,
                        unsigned seed = 1234);

    std::vector<double> forward(const std::vector<double>& input) const;

    // One SGD step on a single (input, target) pair using MSE loss. Returns
    // the pre-update squared error, useful for tracking training progress.
    double trainStep(const std::vector<double>& input, const std::vector<double>& target,
                      double learningRate);

    double trainEpoch(const std::vector<std::vector<double>>& inputs,
                       const std::vector<std::vector<double>>& targets, double learningRate);

    // Flattened weights+biases, for GA-based (gradient-free) weight tuning.
    std::vector<double> parameters() const;
    void setParameters(const std::vector<double>& params);
    std::size_t parameterCount() const;

    const std::vector<std::size_t>& layerSizes() const { return layerSizes_; }

private:
    struct Layer {
        std::vector<std::vector<double>> weights;  // weights[out][in]
        std::vector<double> biases;                // biases[out]
        Activation activation;
    };

    static double activate(Activation act, double z);
    static double activateDerivativeFromOutput(Activation act, double activated);

    std::vector<std::size_t> layerSizes_;
    std::vector<Layer> layers_;
};

}  // namespace fuzzylib::nn
