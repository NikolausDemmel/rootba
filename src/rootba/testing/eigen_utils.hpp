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

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <gtest/gtest.h>

namespace rootba {

// //////////////////////////////////////////////////////////////////////////
// macros for matrix near equality (floating point scalar types)
// //////////////////////////////////////////////////////////////////////////

// Helper function for implementing ROOTBA_ASSERT_MAT_NEAR_RELATIVE.
// Handles special cases and values close to zero.
// Source: https://floating-point-gui.de/errors/comparison/
template <class Derived1, class Derived2>
testing::AssertionResult mat_near_relative_pred_format(
    const char* expr1, const char* expr2, const char* precision_expr,
    const Derived1& val1, const Derived2& val2,
    typename Derived1::Scalar precision) {
  constexpr bool is_dense_matrix1 =
      std::is_base_of_v<Eigen::MatrixBase<Derived1>, Derived1>;
  constexpr bool is_dense_matrix2 =
      std::is_base_of_v<Eigen::MatrixBase<Derived2>, Derived2>;
  constexpr bool is_sparse_matrix1 =
      std::is_base_of_v<Eigen::SparseMatrixBase<Derived1>, Derived1>;
  constexpr bool is_sparse_matrix2 =
      std::is_base_of_v<Eigen::SparseMatrixBase<Derived2>, Derived2>;

  static_assert(is_dense_matrix1 || is_sparse_matrix1);
  static_assert(is_dense_matrix2 || is_sparse_matrix2);

  using Scalar = typename Derived1::Scalar;
  static_assert(std::is_same_v<Scalar, typename Derived2::Scalar>);

  const Scalar abs1 = val1.norm();
  const Scalar abs2 = val2.norm();
  const Scalar diff = (val1 - val2).norm();

  bool exactly_equal = false;
  if constexpr (is_dense_matrix1 && is_dense_matrix2) {
    // TODO: how to best implement this for sparse matrices?
    exactly_equal = (val1 == val2);
  }

  if (exactly_equal) {
    // shortcut, handles infinities
    return testing::AssertionSuccess();
  } else if (abs1 == 0 || abs2 == 0 ||
             (abs1 + abs2 < std::numeric_limits<Scalar>::min())) {
    // val1 or val2 is zero or both are extremely close to it
    // relative error is less meaningful here
    Scalar threshold = precision * std::numeric_limits<Scalar>::min();
    if (diff < threshold) {
      return testing::AssertionSuccess();
    } else {
      return testing::AssertionFailure()
             << "The absolute norm of the difference between " << expr1
             << " and " << expr2 << " is " << diff << ", which exceeds ("
             << precision_expr
             << ") * std::numeric_limits<Scalar>::min(), where\n"
             << "the norm of " << expr1 << " evaluates to " << abs1 << ",\n"
             << "the norm of " << expr2 << " evaluates to " << abs2 << ",\n"
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
             << "The relative norm of the difference between " << expr1
             << " and " << expr2 << " is " << rel_diff << " which exceeds "
             << precision_expr << ", where\n"
             << "the norm of " << expr1 << " evaluates to " << abs1 << ",\n"
             << "the norm of " << expr2 << " evaluates to " << abs2 << ",\n"
             << "the absolute norm of the difference is " << diff << ", and\n"
             << precision_expr << " evaluates to " << precision << ".";
    }
  }
}

#define ROOTBA_EXPECT_MAT_NEAR_RELATIVE(val1, val2, precision) \
  EXPECT_PRED_FORMAT3(mat_near_relative_pred_format, val1, val2, precision)

#define ROOTBA_ASSERT_MAT_NEAR_RELATIVE(scalar, val1, val2, precision) \
  ASSERT_PRED_FORMAT3(mat_near_relative_pred_format, val1, val2, precision)

// //////////////////////////////////////////////////////////////////////////
// macros for matrix dimension equality
// //////////////////////////////////////////////////////////////////////////

// Helper function for implementing ROOTBA_ASSERT_MAT_DIM_EQ.
template <class Derived1, class Derived2>
testing::AssertionResult mat_dim_equal_pred_format(
    const char* expr1, const char* expr2,
    const Eigen::EigenBase<Derived1>& val1,
    const Eigen::EigenBase<Derived2>& val2) {
  if (val1.rows() == val2.rows() && val1.cols() == val2.cols()) {
    return testing::AssertionSuccess();
  } else {
    return testing::AssertionFailure()
           << "Expected equality of dimensions of these matrices:\n"
           << "  " << expr1 << "\n"
           << "    Which is: " << val1.rows() << " x " << val1.cols() << "\n"
           << "  " << expr2 << "\n"
           << "    Which is: " << val2.rows() << " x " << val2.cols();
  }
}

#define ROOTBA_EXPECT_MAT_DIM_EQ(val1, val2) \
  EXPECT_PRED_FORMAT2(mat_dim_equal_pred_format, val1, val2)

#define ROOTBA_ASSERT_MAT_DIM_EQ(val1, val2) \
  ASSERT_PRED_FORMAT2(mat_dim_equal_pred_format, val1, val2)

// //////////////////////////////////////////////////////////////////////////
// macros for single matrix tests
// //////////////////////////////////////////////////////////////////////////

// Helper function for implementing ROOTBA_ASSERT_MAT_ISFINITE.
template <class Derived>
testing::AssertionResult mat_isfinite_pred_format(const char* expr,
                                                  const Derived& val) {
  static_assert(std::is_base_of_v<Eigen::MatrixBase<Derived>, Derived> ||
                std::is_base_of_v<Eigen::SparseMatrixBase<Derived>, Derived>);
  const auto sum = val.sum();
  if (std::isfinite(sum)) {
    return testing::AssertionSuccess();
  } else {
    return testing::AssertionFailure()
           << "Expected matrix " << expr
           << " to be finite, but elements sum to " << sum;
  }
}

#define ROOTBA_EXPECT_MAT_ISFINITE(val1) \
  EXPECT_PRED_FORMAT1(mat_isfinite_pred_format, val1)

#define ROOTBA_ASSERT_MAT_ISFINITE(val1) \
  ASSERT_PRED_FORMAT1(mat_isfinite_pred_format, val1)

}  // namespace rootba
