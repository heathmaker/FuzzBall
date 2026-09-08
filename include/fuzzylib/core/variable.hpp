#pragma once

#include <algorithm>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "fuzzylib/core/membership.hpp"

namespace fuzzylib {

// A named fuzzy term ("cold", "warm", ...) backed by a membership function.
struct Term {
    std::string name;
    MembershipFunctionPtr mf;

    double membership(double x) const { return (*mf)(x); }
};

// A linguistic variable is a physical quantity (e.g. "temperature") with a
// crisp universe of discourse [min, max] and a set of named fuzzy terms.
class LinguisticVariable {
public:
    LinguisticVariable(std::string name, double min, double max)
        : name_(std::move(name)), min_(min), max_(max) {
        if (min_ >= max_) throw std::invalid_argument("LinguisticVariable requires min < max");
    }

    const std::string& name() const { return name_; }
    double min() const { return min_; }
    double max() const { return max_; }

    LinguisticVariable& addTerm(const std::string& termName, MembershipFunctionPtr mf) {
        terms_.push_back(Term{termName, std::move(mf)});
        return *this;
    }

    const std::vector<Term>& terms() const { return terms_; }

    const Term& term(const std::string& termName) const {
        for (const auto& t : terms_) {
            if (t.name == termName) return t;
        }
        throw std::out_of_range("Unknown term '" + termName + "' on variable '" + name_ + "'");
    }

    bool hasTerm(const std::string& termName) const {
        return std::any_of(terms_.begin(), terms_.end(),
                            [&termName](const Term& t) { return t.name == termName; });
    }

    // Degree of membership of crisp value x in every term of this variable.
    std::unordered_map<std::string, double> fuzzify(double x) const {
        std::unordered_map<std::string, double> degrees;
        degrees.reserve(terms_.size());
        for (const auto& t : terms_) {
            degrees[t.name] = t.membership(x);
        }
        return degrees;
    }

private:
    std::string name_;
    double min_, max_;
    std::vector<Term> terms_;
};

}  // namespace fuzzylib
