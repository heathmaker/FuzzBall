#pragma once

// Convenience umbrella header pulling in the whole library. Prefer including
// only the specific headers you need in performance-sensitive translation
// units, but for examples/tests this keeps things simple.

#include "fuzzylib/blocks/block.hpp"
#include "fuzzylib/blocks/fis_block.hpp"
#include "fuzzylib/blocks/hdc_block.hpp"
#include "fuzzylib/blocks/nn_block.hpp"
#include "fuzzylib/blocks/pid_block.hpp"
#include "fuzzylib/control/pid.hpp"
#include "fuzzylib/core/fuzzification.hpp"
#include "fuzzylib/core/membership.hpp"
#include "fuzzylib/core/rule.hpp"
#include "fuzzylib/core/tnorm.hpp"
#include "fuzzylib/core/variable.hpp"
#include "fuzzylib/ga/genetic_algorithm.hpp"
#include "fuzzylib/hdc/hypervector.hpp"
#include "fuzzylib/inference/mamdani.hpp"
#include "fuzzylib/inference/sugeno.hpp"
#include "fuzzylib/nn/feedforward.hpp"
