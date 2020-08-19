/**
BSD 3-Clause License

This file is part of the RootBA project.
https://github.com/NikolausDemmel/rootba

Copyright (c) 2021, Nikolaus Demmel.
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

// This file is adapted from Ceres
// (https://github.com/ceres-solver/ceres-solver). Original license:
//
// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: sameeragarwal@google.com (Sameer Agarwal)
//
// Preconditioned Conjugate Gradients based solver for positive
// semidefinite linear systems.

#pragma once

#include <memory>

#include "rootba/cg/utils.hpp"
#include "rootba/util/assert.hpp"

namespace rootba {

template <typename Scalar>
class ConjugateGradientsSolver {
 public:
  struct Options {
    double min_num_iterations = 0;
    int residual_reset_period = 10;
    int max_num_iterations = 50;
  };

  struct PerSolveOptions {
    double r_tolerance = -1;
    double q_tolerance = 0.1;
    std::unique_ptr<Preconditioner<Scalar>> preconditioner;
  };

  struct Summary {
    enum TerminationType {
      LINEAR_SOLVER_NO_CONVERGENCE,
      LINEAR_SOLVER_SUCCESS,
      LINEAR_SOLVER_FAILURE
    };

    TerminationType termination_type;
    std::string message;
    int num_iterations = 0;
  };

  using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  ConjugateGradientsSolver(const Options& options) : options_(options) {}

  Summary solve(const LinearOperator<Scalar>* h,
                const PerSolveOptions& per_solve_options, const VecX& bref,
                VecX& xref) {
    Summary summary;
    summary.termination_type = Summary::LINEAR_SOLVER_NO_CONVERGENCE;
    summary.message = "Maximum number of iterations reached.";
    summary.num_iterations = 0;

    const int num_cols = h->num_cols();
    ROOTBA_ASSERT(num_cols == xref.rows());

    const double norm_b = bref.norm();
    if (norm_b == 0.0) {
      xref.setZero();
      summary.termination_type = Summary::LINEAR_SOLVER_SUCCESS;
      summary.message = "Convergence. |b| = 0.";
      return summary;
    }

    VecX r(num_cols);
    VecX p(num_cols);
    VecX z(num_cols);
    VecX tmp(num_cols);
    VecX q(num_cols);

    const double tol_r = per_solve_options.r_tolerance * norm_b;

    // tmp.setZero();
    // A->RightMultiply(x, tmp.data());
    tmp = h->right_multiply(xref);

    r = bref - tmp;
    double norm_r = r.norm();
    if (options_.min_num_iterations == 0 && norm_r <= tol_r) {
      summary.termination_type = Summary::LINEAR_SOLVER_SUCCESS;

      std::stringstream ss;
      ss << "Convergence. |r| = " << norm_r << " <= " << tol_r;

      summary.message = ss.str();
      return summary;
    }

    double rho = 1.0;

    // Initial value of the quadratic model Q = x'Ax - 2 * b'x.
    double q0 = -1.0 * xref.dot(bref + r);

    for (summary.num_iterations = 1;; ++summary.num_iterations) {
      // Apply preconditioner
      if (per_solve_options.preconditioner != nullptr) {
        // z.setZero();
        // per_solve_options.preconditioner->RightMultiply(r.data(), z.data());

        per_solve_options.preconditioner->solve_assign(r, z);
      } else {
        z = r;
      }

      double last_rho = rho;
      rho = r.dot(z);
      if (is_zero_or_infinity(rho)) {
        summary.termination_type = Summary::LINEAR_SOLVER_FAILURE;

        std::stringstream ss;
        ss << "Numerical failure. rho = r'z = " << rho;

        summary.message = ss.str();
        break;
      }

      if (summary.num_iterations == 1) {
        p = z;
      } else {
        double beta = rho / last_rho;
        if (is_zero_or_infinity(beta)) {
          summary.termination_type = Summary::LINEAR_SOLVER_FAILURE;
          summary.message =
              "Numerical failure. beta = rho_n / rho_{n-1} = %e, "
              "rho_n = %e, rho_{n-1} = %e";
          break;
        }
        p = z + beta * p;
      }

      q = h->right_multiply(p);
      const double pq = p.dot(q);

      if ((pq <= 0) || std::isinf(pq)) {
        summary.termination_type = Summary::LINEAR_SOLVER_NO_CONVERGENCE;

        std::stringstream ss;
        ss << "Matrix is indefinite, no more progress can be made. p'q = " << pq
           << ". |p| = " << p.norm() << ", |q| = " << q.norm();

        summary.message = ss.str();

        break;
      }

      const double alpha = rho / pq;
      if (std::isinf(alpha)) {
        summary.termination_type = Summary::LINEAR_SOLVER_FAILURE;
        summary.message =
            "Numerical failure. alpha = rho / pq = %e, rho = %e, pq = %e.";
        break;
      }

      xref = xref + alpha * p;

      // Ideally we would just use the update r = r - alpha*q to keep
      // track of the residual vector. However this estimate tends to
      // drift over time due to round off errors. Thus every
      // residual_reset_period iterations, we calculate the residual as
      // r = b - Ax. We do not do this every iteration because this
      // requires an additional matrix vector multiply which would
      // double the complexity of the CG algorithm.
      if (summary.num_iterations % options_.residual_reset_period == 0) {
        tmp = h->right_multiply(xref);
        r = bref - tmp;
      } else {
        r = r - alpha * q;
      }

      // Quadratic model based termination.
      //   Q1 = x'Ax - 2 * b' x.
      const double q1 = -1.0 * xref.dot(bref + r);

      // For PSD matrices A, let
      //
      //   Q(x) = x'Ax - 2b'x
      //
      // be the cost of the quadratic function defined by A and b. Then,
      // the solver terminates at iteration i if
      //
      //   i * (Q(x_i) - Q(x_i-1)) / Q(x_i) < q_tolerance.
      //
      // This termination criterion is more useful when using CG to
      // solve the Newton step. This particular convergence test comes
      // from Stephen Nash's work on truncated Newton
      // methods. References:
      //
      //   1. Stephen G. Nash & Ariela Sofer, Assessing A Search
      //   Direction Within A Truncated Newton Method, Operation
      //   Research Letters 9(1990) 219-221.
      //
      //   2. Stephen G. Nash, A Survey of Truncated Newton Methods,
      //   Journal of Computational and Applied Mathematics,
      //   124(1-2), 45-59, 2000.
      //
      const double zeta = summary.num_iterations * (q1 - q0) / q1;
      if (zeta < per_solve_options.q_tolerance &&
          summary.num_iterations >= options_.min_num_iterations) {
        summary.termination_type = Summary::LINEAR_SOLVER_SUCCESS;

        std::stringstream ss;
        ss << "Iteration: " << summary.num_iterations
           << " Convergence. zeta = " << zeta << " < "
           << per_solve_options.q_tolerance << ". |r| = " << r.norm();

        summary.message = ss.str();

        break;
      }
      q0 = q1;

      // Residual based termination.
      norm_r = r.norm();
      if (norm_r <= tol_r &&
          summary.num_iterations >= options_.min_num_iterations) {
        summary.termination_type = Summary::LINEAR_SOLVER_SUCCESS;
        std::stringstream ss;
        ss << "Iteration: " << summary.num_iterations
           << " Convergence. |r| = " << norm_r << " <= " << tol_r;

        summary.message = ss.str();
        break;
      }

      if (summary.num_iterations >= options_.max_num_iterations) {
        break;
      }
    }

    return summary;
  }

 private:
  Options options_;
};

}  // namespace rootba
