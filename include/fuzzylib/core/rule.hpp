#pragma once

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "fuzzylib/core/tnorm.hpp"

namespace fuzzylib {

// Degrees of membership for every (variable, term) pair, computed once per
// evaluation cycle from the crisp inputs, and shared by the whole rule base.
using FuzzificationTable = std::unordered_map<std::string, std::unordered_map<std::string, double>>;

// Linguistic hedges modify a membership degree before it is used in a rule,
// e.g. "very hot" concentrates the set, "somewhat hot" dilates it.
enum class Hedge { None, Very, ExtremelyVery, Somewhat, Slightly };

inline double applyHedge(Hedge h, double degree) {
    switch (h) {
        case Hedge::None:
            return degree;
        case Hedge::Very:
            return degree * degree;
        case Hedge::ExtremelyVery:
            return degree * degree * degree;
        case Hedge::Somewhat:
            return std::sqrt(degree);
        case Hedge::Slightly:
            return std::pow(degree, 1.0 / 3.0);
    }
    return degree;
}

// Base class for antecedent expression trees: leaves reference a
// (variable, term) pair; internal nodes combine children with AND/OR/NOT.
class Antecedent {
public:
    virtual ~Antecedent() = default;
    virtual double evaluate(const FuzzificationTable& table, const TNorm& tnorm) const = 0;
    virtual std::unique_ptr<Antecedent> clone() const = 0;
};
using AntecedentPtr = std::unique_ptr<Antecedent>;

namespace antecedent {

class Is final : public Antecedent {
public:
    Is(std::string variable, std::string term, Hedge hedge = Hedge::None)
        : variable_(std::move(variable)), term_(std::move(term)), hedge_(hedge) {}

    double evaluate(const FuzzificationTable& table, const TNorm&) const override {
        auto varIt = table.find(variable_);
        if (varIt == table.end()) {
            throw std::out_of_range("Antecedent references unknown variable '" + variable_ + "'");
        }
        auto termIt = varIt->second.find(term_);
        if (termIt == varIt->second.end()) {
            throw std::out_of_range("Antecedent references unknown term '" + term_ + "' on '" +
                                     variable_ + "'");
        }
        return applyHedge(hedge_, termIt->second);
    }

    std::unique_ptr<Antecedent> clone() const override { return std::make_unique<Is>(*this); }

private:
    std::string variable_, term_;
    Hedge hedge_;
};

class And final : public Antecedent {
public:
    And(AntecedentPtr lhs, AntecedentPtr rhs) : lhs_(std::move(lhs)), rhs_(std::move(rhs)) {}

    double evaluate(const FuzzificationTable& table, const TNorm& tnorm) const override {
        return tnorm.and_op(lhs_->evaluate(table, tnorm), rhs_->evaluate(table, tnorm));
    }

    std::unique_ptr<Antecedent> clone() const override {
        return std::make_unique<And>(lhs_->clone(), rhs_->clone());
    }

private:
    AntecedentPtr lhs_, rhs_;
};

class Or final : public Antecedent {
public:
    Or(AntecedentPtr lhs, AntecedentPtr rhs) : lhs_(std::move(lhs)), rhs_(std::move(rhs)) {}

    double evaluate(const FuzzificationTable& table, const TNorm& tnorm) const override {
        return tnorm.or_op(lhs_->evaluate(table, tnorm), rhs_->evaluate(table, tnorm));
    }

    std::unique_ptr<Antecedent> clone() const override {
        return std::make_unique<Or>(lhs_->clone(), rhs_->clone());
    }

private:
    AntecedentPtr lhs_, rhs_;
};

class Not final : public Antecedent {
public:
    explicit Not(AntecedentPtr child) : child_(std::move(child)) {}

    double evaluate(const FuzzificationTable& table, const TNorm& tnorm) const override {
        return 1.0 - child_->evaluate(table, tnorm);
    }

    std::unique_ptr<Antecedent> clone() const override {
        return std::make_unique<Not>(child_->clone());
    }

private:
    AntecedentPtr child_;
};

inline AntecedentPtr is(std::string variable, std::string term, Hedge hedge = Hedge::None) {
    return std::make_unique<Is>(std::move(variable), std::move(term), hedge);
}

}  // namespace antecedent

// operator&/operator|/operator~ on AntecedentPtr must live in namespace
// fuzzylib (not fuzzylib::antecedent) so that argument-dependent lookup at
// call sites like `antecedent::is(...) & antecedent::is(...)` finds them:
// ADL only searches the namespaces associated with the operands' types, and
// the associated namespace of std::unique_ptr<Antecedent> is the namespace
// of Antecedent itself (fuzzylib), not the nested antecedent namespace.
inline AntecedentPtr operator&(AntecedentPtr lhs, AntecedentPtr rhs) {
    return std::make_unique<antecedent::And>(std::move(lhs), std::move(rhs));
}
inline AntecedentPtr operator|(AntecedentPtr lhs, AntecedentPtr rhs) {
    return std::make_unique<antecedent::Or>(std::move(lhs), std::move(rhs));
}
inline AntecedentPtr operator~(AntecedentPtr child) {
    return std::make_unique<antecedent::Not>(std::move(child));
}

// Mamdani-style consequent: "output variable IS term", optionally weighted.
struct MamdaniConsequent {
    std::string variable;
    std::string term;
    double weight = 1.0;
};

// Takagi-Sugeno-Kang consequent: output = constant + sum(coeff_i * input_i).
// An empty `coefficients` map with only `constant` set is a zero-order (TS0)
// rule; populating coefficients makes it a first-order (TS1) rule.
struct SugenoConsequent {
    std::string variable;
    double constant = 0.0;
    std::unordered_map<std::string, double> coefficients;

    double evaluate(const std::unordered_map<std::string, double>& inputs) const {
        double z = constant;
        for (const auto& [name, coeff] : coefficients) {
            auto it = inputs.find(name);
            if (it == inputs.end()) {
                throw std::out_of_range("SugenoConsequent references unknown input '" + name + "'");
            }
            z += coeff * it->second;
        }
        return z;
    }
};

// A single fuzzy rule: IF <antecedent> THEN <consequent(s)>, with an
// optional rule weight/confidence in [0, 1] applied to the firing strength.
class Rule {
public:
    explicit Rule(AntecedentPtr antecedent, double weight = 1.0)
        : antecedent_(std::move(antecedent)), weight_(weight) {}

    // Returns Rule&& (not Rule&) so that the common fluent-builder pattern
    // `Rule(antecedent).then(consequent)` yields an xvalue: Rule is
    // move-only (it owns a unique_ptr<Antecedent>), and a by-value
    // RuleBase::add(Rule) parameter can only bind to that expression if it
    // is treated as movable rather than as a plain lvalue reference.
    Rule&& then(MamdaniConsequent c) {
        mamdaniConsequents_.push_back(std::move(c));
        return std::move(*this);
    }
    Rule&& then(SugenoConsequent c) {
        sugenoConsequents_.push_back(std::move(c));
        return std::move(*this);
    }

    double firingStrength(const FuzzificationTable& table, const TNorm& tnorm) const {
        return weight_ * antecedent_->evaluate(table, tnorm);
    }

    const std::vector<MamdaniConsequent>& mamdaniConsequents() const { return mamdaniConsequents_; }
    const std::vector<SugenoConsequent>& sugenoConsequents() const { return sugenoConsequents_; }
    double weight() const { return weight_; }

private:
    AntecedentPtr antecedent_;
    double weight_;
    std::vector<MamdaniConsequent> mamdaniConsequents_;
    std::vector<SugenoConsequent> sugenoConsequents_;
};

// An ordered collection of rules that share a fuzzification table.
class RuleBase {
public:
    RuleBase& add(Rule rule) {
        rules_.push_back(std::move(rule));
        return *this;
    }
    const std::vector<Rule>& rules() const { return rules_; }

private:
    std::vector<Rule> rules_;
};

}  // namespace fuzzylib
