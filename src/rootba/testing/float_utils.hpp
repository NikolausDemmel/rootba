/**
BSD 3-Clause License

This file is part of the RootBA project.
https://github.com/NikolausDemmel/rootba

Copyright (c) 2021-2023, Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#pragma once

#include <gtest/gtest.h>

namespace rootba {

// //////////////////////////////////////////////////////////////////////////
// helpers for floating point precision thresholds
// //////////////////////////////////////////////////////////////////////////

template <class Scalar>
Scalar precision_float_or_double(float float_precision,
                                 double double_precision) {
  (void)float_precision;
  (void)double_precision;
  if constexpr (std::is_same_v<float, Scalar>) {
    return float_precision;
  } else if constexpr (std::is_same_v<double, Scalar>) {
    return double_precision;
  } else {
    // "!sizeof(Scalar)" is a way to encode "false" such that it depends on the
    // template parameter
    static_assert(!sizeof(Scalar), "only float and double are supported");
  }
}

template <class Scalar>
Scalar default_test_precision() {
  // For reference:
  // Eigen::NumTraits<Scalar>::dummy_precision() gives:
  //   float -> 1e-5F
  //  double -> 1e-12
  return precision_float_or_double<Scalar>(1e-5F, 1e-12);
}

// //////////////////////////////////////////////////////////////////////////
// comparison of floating point numbers with relative error
// //////////////////////////////////////////////////////////////////////////

// Return true if floating point numbers are equal up to given precision
// (relative error). Handles special cases and values close to zero.
// Source: https://floating-point-gui.de/errors/comparison/
template <class Scalar>
static bool nearly_equal(const Scalar a, const Scalar b,
                         const Scalar precision) {
  const Scalar abs_a = std::abs(a);
  const Scalar abs_b = std::abs(b);
  const Scalar diff = std::abs(a - b);

  if (a == b) {
    // shortcut, handles infinities
    return true;
  } else if (a == 0 || b == 0 ||
             (abs_a + abs_b < std::numeric_limits<Scalar>::min())) {
    // a or b is zero or both are extremely close to it
    // relative error is less meaningful here
    return diff < (precision * std::numeric_limits<Scalar>::min());
  } else {
    // use relative error
    Scalar rel_diff =
        diff / std::min((abs_a + abs_b), std::numeric_limits<Scalar>::max());
    return rel_diff < precision;
  }
}

// //////////////////////////////////////////////////////////////////////////
// macros for floating point comparison
// //////////////////////////////////////////////////////////////////////////

// Helper function for implementing ROOTBA_EXPECT_NEAR_RELATIVE.
// Handles special cases and values close to zero.
// Source: https://floating-point-gui.de/errors/comparison/
template <class Scalar>
testing::AssertionResult float_near_relative_pred_format(
    const char* expr1, const char* expr2, const char* precision_expr,
    Scalar val1, Scalar val2, Scalar precision) {
  const Scalar abs1 = std::abs(val1);
  const Scalar abs2 = std::abs(val2);
  const Scalar diff = std::abs(val1 - val2);

  if (val1 == val2) {
    // shortcut, handles infinities
    return testing::AssertionSuccess();
  } else if (val1 == 0 || val2 == 0 ||
             (abs1 + abs2 < std::numeric_limits<Scalar>::min())) {
    // val1 or val2 is zero or both are extremely close to it
    // relative error is less meaningful here
    Scalar threshold = precision * std::numeric_limits<Scalar>::min();
    if (diff < threshold) {
      return testing::AssertionSuccess();
    } else {
      return testing::AssertionFailure()
             << "The absolute difference between " << expr1 << " and " << expr2
             << " is " << diff << ", which exceeds (" << precision_expr
             << ") * std::numeric_limits<Scalar>::min(), where\n"
             << expr1 << " evaluates to " << val1 << ",\n"
             << expr2 << " evaluates to " << val2 << ",\n"
             << precision_expr << " evaluates to " << precision << ", and\n"
             << "(" << precision_expr
             << ") * std::numeric_limits<Scalar>::min() evaluates to "
             << threshold << " (one or both values are very close to 0).";
    }
  } else {
    // use relative error
    Scalar rel_diff =
        diff / std::min((abs1 + abs2), std::numeric_limits<Scalar>::max());
    if (rel_diff < precision) {
      return testing::AssertionSuccess();
    } else {
      return testing::AssertionFailure()
             << "The relative difference between " << expr1 << " and " << expr2
             << " is " << rel_diff << " which exceeds " << precision_expr
             << ", where\n"
             << expr1 << " evaluates to " << val1 << ",\n"
             << expr2 << " evaluates to " << val2 << ",\n"
             << "the absolute difference is " << diff << ", and\n"
             << precision_expr << " evaluates to " << precision << ".";
    }
  }
}

#define ROOTBA_EXPECT_NEAR_RELATIVE(val1, val2, precision) \
  EXPECT_PRED_FORMAT3(float_near_relative_pred_format, val1, val2, precision)

#define ROOTBA_ASSERT_NEAR_RELATIVE(scalar, val1, val2, precision) \
  ASSERT_PRED_FORMAT3(float_near_relative_pred_format, val1, val2, precision)

}  // namespace rootba
