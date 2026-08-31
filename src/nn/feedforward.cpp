#include "fuzzylib/nn/feedforward.hpp"

#include <cmath>
#include <random>
#include <stdexcept>

namespace fuzzylib::nn {

FeedForwardNetwork::FeedForwardNetwork(std::vector<std::size_t> layerSizes,
                                         std::vector<Activation> activations, unsigned seed)
    : layerSizes_(std::move(layerSizes)) {
    if (layerSizes_.size() < 2 || activations.size() != layerSizes_.size() - 1) {
        throw std::invalid_argument("FeedForwardNetwork: activations.size() must equal layers-1");
    }

    std::mt19937 rng(seed);
    layers_.resize(activations.size());
    for (std::size_t l = 0; l < layers_.size(); ++l) {
        const std::size_t in = layerSizes_[l];
        const std::size_t out = layerSizes_[l + 1];
        // Xavier/Glorot-style initialization keeps activations well-scaled
        // regardless of layer width.
        const double limit = std::sqrt(6.0 / static_cast<double>(in + out));
        std::uniform_real_distribution<double> dist(-limit, limit);

        Layer& layer = layers_[l];
        layer.activation = activations[l];
        layer.weights.assign(out, std::vector<double>(in));
        layer.biases.assign(out, 0.0);
        for (std::size_t o = 0; o < out; ++o) {
            for (std::size_t i = 0; i < in; ++i) layer.weights[o][i] = dist(rng);
        }
    }
}

double FeedForwardNetwork::activate(Activation act, double z) {
    switch (act) {
        case Activation::Linear:
            return z;
        case Activation::ReLU:
            return z > 0.0 ? z : 0.0;
        case Activation::Tanh:
            return std::tanh(z);
        case Activation::Sigmoid:
            return 1.0 / (1.0 + std::exp(-z));
    }
    return z;
}

double FeedForwardNetwork::activateDerivativeFromOutput(Activation act, double activated) {
    switch (act) {
        case Activation::Linear:
            return 1.0;
        case Activation::ReLU:
            return activated > 0.0 ? 1.0 : 0.0;
        case Activation::Tanh:
            return 1.0 - activated * activated;
        case Activation::Sigmoid:
            return activated * (1.0 - activated);
    }
    return 1.0;
}

std::vector<double> FeedForwardNetwork::forward(const std::vector<double>& input) const {
    std::vector<double> activations = input;
    for (const Layer& layer : layers_) {
        std::vector<double> next(layer.biases.size());
        for (std::size_t o = 0; o < layer.weights.size(); ++o) {
            double z = layer.biases[o];
            for (std::size_t i = 0; i < activations.size(); ++i) z += layer.weights[o][i] * activations[i];
            next[o] = activate(layer.activation, z);
        }
        activations = std::move(next);
    }
    return activations;
}

double FeedForwardNetwork::trainStep(const std::vector<double>& input, const std::vector<double>& target,
                                       double learningRate) {
    // Forward pass, caching every layer's activations for backprop.
    std::vector<std::vector<double>> activationsPerLayer;
    activationsPerLayer.reserve(layers_.size() + 1);
    activationsPerLayer.push_back(input);

    for (const Layer& layer : layers_) {
        const std::vector<double>& prev = activationsPerLayer.back();
        std::vector<double> next(layer.biases.size());
        for (std::size_t o = 0; o < layer.weights.size(); ++o) {
            double z = layer.biases[o];
            for (std::size_t i = 0; i < prev.size(); ++i) z += layer.weights[o][i] * prev[i];
            next[o] = activate(layer.activation, z);
        }
        activationsPerLayer.push_back(std::move(next));
    }

    const std::vector<double>& output = activationsPerLayer.back();
    double squaredError = 0.0;
    std::vector<double> delta(output.size());
    for (std::size_t i = 0; i < output.size(); ++i) {
        const double err = output[i] - target[i];
        squaredError += err * err;
        delta[i] = err * activateDerivativeFromOutput(layers_.back().activation, output[i]);
    }

    // Backpropagate layer by layer, updating weights in place.
    for (std::size_t l = layers_.size(); l-- > 0;) {
        Layer& layer = layers_[l];
        const std::vector<double>& prevActivations = activationsPerLayer[l];
        std::vector<double> nextDelta(prevActivations.size(), 0.0);

        for (std::size_t o = 0; o < layer.weights.size(); ++o) {
            for (std::size_t i = 0; i < prevActivations.size(); ++i) {
                nextDelta[i] += delta[o] * layer.weights[o][i];
                layer.weights[o][i] -= learningRate * delta[o] * prevActivations[i];
            }
            layer.biases[o] -= learningRate * delta[o];
        }

        if (l > 0) {
            const Activation prevAct = layers_[l - 1].activation;
            for (std::size_t i = 0; i < nextDelta.size(); ++i) {
                nextDelta[i] *= activateDerivativeFromOutput(prevAct, prevActivations[i]);
            }
        }
        delta = std::move(nextDelta);
    }

    return squaredError;
}

double FeedForwardNetwork::trainEpoch(const std::vector<std::vector<double>>& inputs,
                                        const std::vector<std::vector<double>>& targets,
                                        double learningRate) {
    double totalError = 0.0;
    for (std::size_t i = 0; i < inputs.size(); ++i) {
        totalError += trainStep(inputs[i], targets[i], learningRate);
    }
    return totalError / static_cast<double>(inputs.empty() ? 1 : inputs.size());
}

std::vector<double> FeedForwardNetwork::parameters() const {
    std::vector<double> flat;
    flat.reserve(parameterCount());
    for (const Layer& layer : layers_) {
        for (const auto& row : layer.weights) flat.insert(flat.end(), row.begin(), row.end());
        flat.insert(flat.end(), layer.biases.begin(), layer.biases.end());
    }
    return flat;
}

void FeedForwardNetwork::setParameters(const std::vector<double>& params) {
    if (params.size() != parameterCount()) {
        throw std::invalid_argument("setParameters: size mismatch");
    }
    std::size_t idx = 0;
    for (Layer& layer : layers_) {
        for (auto& row : layer.weights) {
            for (auto& w : row) w = params[idx++];
        }
        for (auto& b : layer.biases) b = params[idx++];
    }
}

std::size_t FeedForwardNetwork::parameterCount() const {
    std::size_t count = 0;
    for (const Layer& layer : layers_) {
        for (const auto& row : layer.weights) count += row.size();
        count += layer.biases.size();
    }
    return count;
}

}  // namespace fuzzylib::nn
