#pragma once

#include <memory>
#include <utility>

#include "fuzzylib/blocks/block.hpp"
#include "fuzzylib/inference/mamdani.hpp"
#include "fuzzylib/inference/sugeno.hpp"

namespace fuzzylib::blocks {

// Wraps a Mamdani FIS as an IBlock so it can be composed in a Pipeline/Blend.
class MamdaniBlock final : public IBlock {
public:
    explicit MamdaniBlock(std::shared_ptr<MamdaniEngine> engine,
                            DefuzzMethod method = DefuzzMethod::Centroid)
        : engine_(std::move(engine)), method_(method) {}

    Signals evaluate(const Signals& inputs) override { return engine_->evaluate(inputs, method_); }

    std::string name() const override { return "mamdani"; }

    MamdaniEngine& engine() { return *engine_; }

private:
    std::shared_ptr<MamdaniEngine> engine_;
    DefuzzMethod method_;
};

// Wraps a Takagi-Sugeno-Kang FIS as an IBlock.
class SugenoBlock final : public IBlock {
public:
    explicit SugenoBlock(std::shared_ptr<SugenoEngine> engine) : engine_(std::move(engine)) {}

    Signals evaluate(const Signals& inputs) override { return engine_->evaluate(inputs); }

    std::string name() const override { return "sugeno"; }

    SugenoEngine& engine() { return *engine_; }

private:
    std::shared_ptr<SugenoEngine> engine_;
};

}  // namespace fuzzylib::blocks
