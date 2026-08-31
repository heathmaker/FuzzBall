#include "fuzzylib/blocks/block.hpp"

#include <memory>

#include "fuzzylib/blocks/fis_block.hpp"
#include "fuzzylib/blocks/hdc_block.hpp"
#include "fuzzylib/blocks/nn_block.hpp"
#include "fuzzylib/blocks/pid_block.hpp"
#include "fuzzylib/core/membership.hpp"
#include "minitest.hpp"

using namespace fuzzylib;
using namespace fuzzylib::blocks;

namespace {
class DoublerBlock final : public IBlock {
public:
    Signals evaluate(const Signals& inputs) override {
        Signals out;
        out["doubled"] = inputs.at("x") * 2.0;
        return out;
    }
};
}  // namespace

TEST_CASE(pipeline_threads_signals_between_stages) {
    Pipeline pipeline;
    pipeline.add("double", std::make_shared<DoublerBlock>());
    pipeline.add("double_again", std::make_shared<DoublerBlock>() /* reads 'x', ignores 'doubled' */);

    Signals result = pipeline.evaluate({{"x", 3.0}});
    CHECK_NEAR(result.at("doubled"), 6.0, 1e-9);
    CHECK_NEAR(result.at("x"), 3.0, 1e-9);  // original input still present downstream
}

TEST_CASE(blend_computes_static_weighted_average) {
    class ConstantBlock final : public IBlock {
    public:
        explicit ConstantBlock(double v) : v_(v) {}
        Signals evaluate(const Signals&) override { return {{"v", v_}}; }

    private:
        double v_;
    };

    Blend blend({BlendMember{std::make_shared<ConstantBlock>(0.0), "v", "", 1.0},
                  BlendMember{std::make_shared<ConstantBlock>(10.0), "v", "", 3.0}},
                 "blended");
    const Signals result = blend.evaluate({});
    CHECK_NEAR(result.at("blended"), 7.5, 1e-9);  // (1*0 + 3*10) / 4
}

TEST_CASE(blend_reads_dynamic_weight_from_upstream_signal) {
    class ConstantBlock final : public IBlock {
    public:
        explicit ConstantBlock(double v) : v_(v) {}
        Signals evaluate(const Signals&) override { return {{"v", v_}}; }

    private:
        double v_;
    };

    Blend blend({BlendMember{std::make_shared<ConstantBlock>(0.0), "v", "wA", 1.0},
                  BlendMember{std::make_shared<ConstantBlock>(100.0), "v", "wB", 1.0}},
                 "blended");
    const Signals result = blend.evaluate({{"wA", 0.0}, {"wB", 1.0}});
    CHECK_NEAR(result.at("blended"), 100.0, 1e-9);
}

TEST_CASE(pid_block_reads_named_signals) {
    control::PIDController::Config cfg;
    cfg.kp = 2.0;
    PIDBlock block(cfg, /*dt=*/0.1);
    Signals out = block.evaluate({{"setpoint", 5.0}, {"measurement", 2.0}});
    CHECK_NEAR(out.at("output"), 6.0, 1e-9);
}

TEST_CASE(mamdani_block_wraps_engine) {
    auto engine = std::make_shared<MamdaniEngine>();
    LinguisticVariable x("x", 0.0, 10.0);
    x.addTerm("any", mf::make<mf::Constant>(1.0));
    LinguisticVariable y("y", 0.0, 10.0);
    y.addTerm("mid", mf::make<mf::Triangular>(2.0, 5.0, 8.0));
    engine->addInput(x).addOutput(y);
    engine->rules().add(Rule(antecedent::is("x", "any")).then(MamdaniConsequent{"y", "mid"}));

    MamdaniBlock block(engine);
    Signals out = block.evaluate({{"x", 1.0}});
    CHECK_NEAR(out.at("y"), 5.0, 0.1);
}

TEST_CASE(hdc_prototype_block_matches_closest_regime) {
    auto space = std::make_shared<hdc::HDCSpace>(3000, 11);
    HDCPrototypeBlock block(space);
    block.addInput("density", 0.0, 1.0, 10);
    block.addPrototype("sparse", {{"density", 0.0}}, /*outputValue=*/1.0);
    block.addPrototype("crowded", {{"density", 1.0}}, /*outputValue=*/0.1);

    const Signals sparseResult = block.evaluate({{"density", 0.02}});
    const Signals crowdedResult = block.evaluate({{"density", 0.95}});
    CHECK_NEAR(sparseResult.at("value"), 1.0, 1e-9);
    CHECK_NEAR(crowdedResult.at("value"), 0.1, 1e-9);
}

int main() { return minitest::run_all_tests(); }
