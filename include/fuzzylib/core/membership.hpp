#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>

namespace fuzzylib {

// Abstract membership function mu(x) -> [0, 1].
class MembershipFunction {
public:
    virtual ~MembershipFunction() = default;
    virtual double operator()(double x) const = 0;
    virtual std::unique_ptr<MembershipFunction> clone() const = 0;
};

using MembershipFunctionPtr = std::shared_ptr<MembershipFunction>;

namespace mf {

// Triangular membership function defined by (a <= b <= c).
class Triangular final : public MembershipFunction {
public:
    Triangular(double a, double b, double c) : a_(a), b_(b), c_(c) {
        if (!(a_ <= b_ && b_ <= c_)) {
            throw std::invalid_argument("Triangular requires a <= b <= c");
        }
    }

    double operator()(double x) const override {
        if (x <= a_ || x >= c_) return 0.0;
        if (x == b_) return 1.0;
        if (x < b_) return (b_ == a_) ? 1.0 : (x - a_) / (b_ - a_);
        return (c_ == b_) ? 1.0 : (c_ - x) / (c_ - b_);
    }

    std::unique_ptr<MembershipFunction> clone() const override {
        return std::make_unique<Triangular>(*this);
    }

private:
    double a_, b_, c_;
};

// Trapezoidal membership function defined by (a <= b <= c <= d).
class Trapezoidal final : public MembershipFunction {
public:
    Trapezoidal(double a, double b, double c, double d) : a_(a), b_(b), c_(c), d_(d) {
        if (!(a_ <= b_ && b_ <= c_ && c_ <= d_)) {
            throw std::invalid_argument("Trapezoidal requires a <= b <= c <= d");
        }
    }

    double operator()(double x) const override {
        if (x <= a_ || x >= d_) return 0.0;
        if (x >= b_ && x <= c_) return 1.0;
        if (x < b_) return (b_ == a_) ? 1.0 : (x - a_) / (b_ - a_);
        return (d_ == c_) ? 1.0 : (d_ - x) / (d_ - c_);
    }

    std::unique_ptr<MembershipFunction> clone() const override {
        return std::make_unique<Trapezoidal>(*this);
    }

private:
    double a_, b_, c_, d_;
};

// Gaussian membership function: exp(-(x-mean)^2 / (2*sigma^2)).
class Gaussian final : public MembershipFunction {
public:
    Gaussian(double mean, double sigma) : mean_(mean), sigma_(sigma) {
        if (sigma_ <= 0.0) throw std::invalid_argument("Gaussian requires sigma > 0");
    }

    double operator()(double x) const override {
        const double z = (x - mean_) / sigma_;
        return std::exp(-0.5 * z * z);
    }

    std::unique_ptr<MembershipFunction> clone() const override {
        return std::make_unique<Gaussian>(*this);
    }

private:
    double mean_, sigma_;
};

// Generalized bell membership function: 1 / (1 + |((x-c)/a)|^(2b)).
class GeneralizedBell final : public MembershipFunction {
public:
    GeneralizedBell(double a, double b, double c) : a_(a), b_(b), c_(c) {
        if (a_ == 0.0) throw std::invalid_argument("GeneralizedBell requires a != 0");
    }

    double operator()(double x) const override {
        const double base = std::fabs((x - c_) / a_);
        return 1.0 / (1.0 + std::pow(base, 2.0 * b_));
    }

    std::unique_ptr<MembershipFunction> clone() const override {
        return std::make_unique<GeneralizedBell>(*this);
    }

private:
    double a_, b_, c_;
};

// Sigmoidal membership function: 1 / (1 + exp(-a*(x-c))).
class Sigmoid final : public MembershipFunction {
public:
    Sigmoid(double a, double c) : a_(a), c_(c) {}

    double operator()(double x) const override {
        return 1.0 / (1.0 + std::exp(-a_ * (x - c_)));
    }

    std::unique_ptr<MembershipFunction> clone() const override {
        return std::make_unique<Sigmoid>(*this);
    }

private:
    double a_, c_;
};

// S-shaped membership function, rises from 0 at 'a' to 1 at 'b'.
class SShape final : public MembershipFunction {
public:
    SShape(double a, double b) : a_(a), b_(b) {
        if (a_ > b_) throw std::invalid_argument("SShape requires a <= b");
    }

    double operator()(double x) const override {
        if (x <= a_) return 0.0;
        if (x >= b_) return 1.0;
        const double mid = (a_ + b_) / 2.0;
        if (x <= mid) {
            const double t = (x - a_) / (b_ - a_);
            return 2.0 * t * t;
        }
        const double t = (b_ - x) / (b_ - a_);
        return 1.0 - 2.0 * t * t;
    }

    std::unique_ptr<MembershipFunction> clone() const override {
        return std::make_unique<SShape>(*this);
    }

private:
    double a_, b_;
};

// Z-shaped membership function, falls from 1 at 'a' to 0 at 'b'.
class ZShape final : public MembershipFunction {
public:
    ZShape(double a, double b) : a_(a), b_(b) {
        if (a_ > b_) throw std::invalid_argument("ZShape requires a <= b");
    }

    double operator()(double x) const override {
        SShape mirror(a_, b_);
        return 1.0 - mirror(x);
    }

    std::unique_ptr<MembershipFunction> clone() const override {
        return std::make_unique<ZShape>(*this);
    }

private:
    double a_, b_;
};

// Singleton membership function: 1 exactly at 'value', 0 elsewhere (within epsilon).
class Singleton final : public MembershipFunction {
public:
    explicit Singleton(double value, double epsilon = 1e-9) : value_(value), epsilon_(epsilon) {}

    double operator()(double x) const override {
        return std::fabs(x - value_) <= epsilon_ ? 1.0 : 0.0;
    }

    std::unique_ptr<MembershipFunction> clone() const override {
        return std::make_unique<Singleton>(*this);
    }

private:
    double value_, epsilon_;
};

// Constant membership function, useful for "don't care" terms or testing.
class Constant final : public MembershipFunction {
public:
    explicit Constant(double value) : value_(value) {}

    double operator()(double /*x*/) const override { return value_; }

    std::unique_ptr<MembershipFunction> clone() const override {
        return std::make_unique<Constant>(*this);
    }

private:
    double value_;
};

// Wraps an arbitrary callable as a membership function, for custom shapes.
class Custom final : public MembershipFunction {
public:
    explicit Custom(std::function<double(double)> fn) : fn_(std::move(fn)) {}

    double operator()(double x) const override { return fn_(x); }

    std::unique_ptr<MembershipFunction> clone() const override {
        return std::make_unique<Custom>(*this);
    }

private:
    std::function<double(double)> fn_;
};

template <typename T, typename... Args>
MembershipFunctionPtr make(Args&&... args) {
    return std::make_shared<T>(std::forward<Args>(args)...);
}

}  // namespace mf
}  // namespace fuzzylib
