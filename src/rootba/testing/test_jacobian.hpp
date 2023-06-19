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
#include <gtest/gtest.h>

namespace rootba {

// //////////////////////////////////////////////////////////////////////////
// constants used in jacobian tests (numeric epsilons, etc...)
// //////////////////////////////////////////////////////////////////////////
template <class Scalar>
struct TestConstants;

template <>
struct TestConstants<double> {
  static constexpr double EPSILON = 1e-8;
  static constexpr double MAX_NORM = 1e-3;
};

template <>
struct TestConstants<float> {
  static constexpr double EPSILON = 1e-2;
  static constexpr double MAX_NORM = 1e-2;
};

// //////////////////////////////////////////////////////////////////////////
// Helper for numeric differention tests for Jacobians
// //////////////////////////////////////////////////////////////////////////
template <typename Derived1, typename Derived2, typename F>
void test_jacobian(
    const std::string& name, const Eigen::MatrixBase<Derived1>& Ja, F func,
    const Eigen::MatrixBase<Derived2>& x0,
    double eps = TestConstants<typename Derived1::Scalar>::EPSILON,
    double max_norm = TestConstants<typename Derived1::Scalar>::MAX_NORM) {
  using Scalar = typename Derived1::Scalar;

  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Jn = Ja;
  Jn.setZero();

  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> inc = x0;
  for (int i = 0; i < Jn.cols(); i++) {
    inc.setZero();
    inc[i] += eps;

    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> fpe = func(x0 + inc);
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> fme = func(x0 - inc);

    Jn.col(i) = (fpe - fme);
  }

  Jn /= (2 * eps);

  EXPECT_TRUE(Ja.allFinite()) << name << ": Ja not finite\n " << Ja;
  EXPECT_TRUE(Jn.allFinite()) << name << ": Jn not finite\n " << Jn;

  if (Jn.isZero(max_norm) && Ja.isZero(max_norm)) {
    EXPECT_TRUE((Jn - Ja).isZero(max_norm))
        << name << ": Ja not equal to Jn(diff norm:" << (Jn - Ja).norm()
        << ")\nJa: (norm: " << Ja.norm() << ")\n"
        << Ja << "\nJn: (norm: " << Jn.norm() << ")\n"
        << Jn;
    //<< "\ndiff:\n" << Jn - Ja;
  } else {
    EXPECT_TRUE(Jn.isApprox(Ja, max_norm))
        << name << ": Ja not equal to Jn (diff norm:" << (Jn - Ja).norm()
        << ", relative thresh: " << max_norm * std::min(Jn.norm(), Ja.norm())
        << ")\nJa: (norm: " << Ja.norm() << ")\n"
        << Ja << "\nJn: (norm: " << Jn.norm() << ")\n"
        << Jn;
    //<< "\ndiff:\n" << Jn - Ja;
  }
}

}  // namespace rootba
