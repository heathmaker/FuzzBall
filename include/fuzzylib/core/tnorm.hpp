#pragma once

#include <algorithm>

namespace fuzzylib {

// A T-norm/T-conorm pair defines how AND/OR combine fuzzy truth values, and
// how implication clips/scales a consequent set by a rule's firing strength.
// Swapping this out changes the "flavor" of inference without touching rules.
struct TNorm {
    double (*and_op)(double, double);
    double (*or_op)(double, double);
    double (*implication)(double firing, double membership);

    static double min_and(double a, double b) { return std::min(a, b); }
    static double max_or(double a, double b) { return std::max(a, b); }
    static double clip_implication(double firing, double membership) {
        return std::min(firing, membership);
    }

    static double product_and(double a, double b) { return a * b; }
    static double probabilistic_or(double a, double b) { return a + b - a * b; }
    static double scale_implication(double firing, double membership) {
        return firing * membership;
    }

    // Classic Mamdani/Zadeh operators: min for AND, max for OR, clipping for implication.
    static TNorm zadeh() { return TNorm{&min_and, &max_or, &clip_implication}; }

    // Algebraic-product operators: product for AND, probabilistic sum for OR,
    // and scaling for implication (common in TSK/Sugeno-style reasoning).
    static TNorm algebraic() { return TNorm{&product_and, &probabilistic_or, &scale_implication}; }
};

// Aggregation of multiple rule outputs onto the same output fuzzy set / value.
struct Aggregator {
    double (*combine)(double, double);
    static double max_agg(double a, double b) { return std::max(a, b); }
    static double sum_agg(double a, double b) { return a + b; }
    static Aggregator max() { return Aggregator{&max_agg}; }
    static Aggregator sum() { return Aggregator{&sum_agg}; }
};

}  // namespace fuzzylib
