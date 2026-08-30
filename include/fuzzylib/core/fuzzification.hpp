#pragma once

#include <stdexcept>
#include <string>
#include <unordered_map>

#include "fuzzylib/core/rule.hpp"
#include "fuzzylib/core/variable.hpp"

namespace fuzzylib {

// Fuzzifies every registered variable against the matching crisp input,
// producing the shared table that rule antecedents evaluate against.
inline FuzzificationTable fuzzifyAll(const std::unordered_map<std::string, LinguisticVariable>& vars,
                                      const std::unordered_map<std::string, double>& inputs) {
    FuzzificationTable table;
    table.reserve(vars.size());
    for (const auto& [name, var] : vars) {
        auto it = inputs.find(name);
        if (it == inputs.end()) {
            throw std::out_of_range("Missing crisp input for variable '" + name + "'");
        }
        table[name] = var.fuzzify(it->second);
    }
    return table;
}

}  // namespace fuzzylib
