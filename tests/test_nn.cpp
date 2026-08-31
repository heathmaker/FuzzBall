#include "fuzzylib/nn/feedforward.hpp"

#include "minitest.hpp"

using namespace fuzzylib::nn;

TEST_CASE(forward_pass_produces_expected_output_size) {
    FeedForwardNetwork net({2, 4, 1}, {Activation::Tanh, Activation::Linear});
    const auto out = net.forward({0.5, -0.5});
    CHECK(out.size() == 1);
}

TEST_CASE(parameters_roundtrip_through_get_set) {
    FeedForwardNetwork net({2, 3, 1}, {Activation::ReLU, Activation::Linear});
    const auto params = net.parameters();
    CHECK(params.size() == net.parameterCount());

    std::vector<double> zeros(params.size(), 0.0);
    net.setParameters(zeros);
    const auto out = net.forward({1.0, 1.0});
    CHECK_NEAR(out[0], 0.0, 1e-9);

    net.setParameters(params);
    CHECK(net.parameters().size() == params.size());
}

TEST_CASE(network_learns_identity_function) {
    // A linear 1-1 network should learn to approximate y = x via SGD.
    FeedForwardNetwork net({1, 1}, {Activation::Linear}, /*seed=*/1);
    std::vector<std::vector<double>> inputs;
    std::vector<std::vector<double>> targets;
    for (int i = -5; i <= 5; ++i) {
        inputs.push_back({static_cast<double>(i)});
        targets.push_back({static_cast<double>(i)});
    }
    double mse = 0.0;
    for (int epoch = 0; epoch < 500; ++epoch) mse = net.trainEpoch(inputs, targets, 0.01);

    CHECK(mse < 0.01);
    CHECK_NEAR(net.forward({3.0})[0], 3.0, 0.2);
}

TEST_CASE(invalid_activation_count_throws) {
    CHECK_THROWS(FeedForwardNetwork({2, 3, 1}, {Activation::Tanh}));
}

int main() { return minitest::run_all_tests(); }
