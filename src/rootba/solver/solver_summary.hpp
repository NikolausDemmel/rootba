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

// This file is adapted from Ceres
// (https://github.com/ceres-solver/ceres-solver). Original license:
//
// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2019 Google Inc. All rights reserved.
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

#pragma once

#include <string>
#include <vector>

#include "rootba/bal/residual_info.hpp"

namespace rootba {

// NOTE(demmeln): These types are similar to (and derived from)
// ceres::IterationSummary and ceres::Solver::Summary. Some of the comments are
// still the same as in Ceres.

enum TerminationType {
  // Minimizer terminated because one of the convergence criterion set
  // by the user was satisfied.
  CONVERGENCE,

  // The solver ran for maximum number of iterations or maximum amount
  // of time specified by the user, but none of the convergence
  // criterion specified by the user were met.
  NO_CONVERGENCE,

  // The minimizer terminated because of an error.
  FAILURE
};

// This struct describes the state of the optimizer after each
// iteration of the minimization.
struct IterationSummary {
  // Current iteration number. Initialization is iteration 0.
  int iteration = 0;

  // Currently used solver, it can be switched only for hybrid solver
  std::string linear_solver_type;

  // Step was numerically valid, i.e., all values are finite and the
  // step reduces the value of the linearized model.
  //
  // Note: step_is_valid is always true when iteration = 0.
  bool step_is_valid = false;

  // Step did not reduce the value of the objective function
  // sufficiently, but it was accepted because of the relaxed
  // acceptance criterion used by the non-monotonic trust region
  // algorithm.
  //
  // Note: step_is_nonmonotonic is always false when iteration = 0;
  bool step_is_nonmonotonic = false;

  // Whether or not the minimizer accepted this step or not. If the
  // ordinary trust region algorithm is used, this means that the
  // relative reduction in the objective function value was greater
  // than Solver::Options::min_relative_decrease. However, if the
  // non-monotonic trust region algorithm is used
  // (Solver::Options:use_nonmonotonic_steps = true), then even if the
  // relative decrease is not sufficient, the algorithm may accept the
  // step and the step is declared successful.
  //
  // Note: step_is_successful is always true when iteration = 0.
  bool step_is_successful = false;

  // Value of the objective function.
  ResidualInfo cost;

  // Change in the value of the objective function in this
  // iteration. This can be positive or negative. Positive means cost was
  // reduced (i.e. change = prev - current)
  ResidualChangeInfo cost_change;

  // Infinity norm of the gradient vector.
  double gradient_max_norm = 0.0;

  // 2-norm of the gradient vector.
  double gradient_norm = 0.0;

  // 2-norm of the size of the step computed by the optimization
  // algorithm.
  double step_norm = 0.0;

  // For trust region algorithms, the ratio of the actual change in
  // cost and the change in the cost of the linearized approximation.
  double relative_decrease = 0.0;

  // Size of the trust region at the end of the current iteration. For
  // the Levenberg-Marquardt algorithm, the regularization parameter
  // mu = 1.0 / trust_region_radius.
  double trust_region_radius = 0.0;

  // Number of iterations taken by the linear solver to solve for the
  // Newton step.
  int linear_solver_iterations = 0;
  std::string linear_solver_message;

  // All times reported below are wall times.

  // logging time is time spent specifically for expensive logging tasks
  double logging_time_in_seconds = 0.0;

  // Time (in seconds) spent inside the minimizer loop in the current
  // iteration.
  double iteration_time_in_seconds = 0.0;

  // Time (in seconds) since the user called optimize().
  double cumulative_time_in_seconds = 0.0;

  // Time (in seconds) spent to compute the LM step (scale_landmark_jacobian +
  // perform_qr + stage 2 + solve + backsubst).
  double step_solver_time_in_seconds = 0.0;

  // Time (in seconds) spent evaluating the residual vector.
  double residual_evaluation_time_in_seconds = 0.0;

  // ----- stage 1 -----

  double jacobian_evaluation_time_in_seconds = 0.0;
  double scale_landmark_jacobian_time_in_seconds = 0.0;
  double perform_qr_time_in_seconds = 0.0;
  double stage1_time_in_seconds = 0.0;

  // ----- stage 2 -----

  double scale_pose_jacobian_time_in_seconds = 0.0;
  double landmark_damping_time_in_seconds = 0.0;
  double compute_preconditioner_time_in_seconds =
      0.0;  //!< may be part of stage 1 if JACOBI
  double compute_gradient_time_in_seconds = 0.0;
  double stage2_time_in_seconds = 0.0;
  double prepare_time_in_seconds = 0.0;

  // ----- post stage 2 -----

  double solve_reduced_system_time_in_seconds = 0.0;
  double back_substitution_time_in_seconds = 0.0;
  double update_cameras_time_in_seconds = 0.0;

  // -----

  // current resident memory in bytes at the end of the iteration
  uint64_t resident_memory = 0;

  // peak resident memory in bytes
  uint64_t resident_memory_peak = 0;
};

struct SolverSummary {
  // A brief one line description of the state of the solver after
  // termination.
  // std::string BriefReport() const;

  // A full multiline description of the state of the solver after
  // termination.
  // std::string FullReport() const;

  // bool IsSolutionUsable() const;

  std::string solver_type;

  // Minimizer summary -------------------------------------------------
  // MinimizerType minimizer_type = TRUST_REGION;

  TerminationType termination_type = FAILURE;

  // Reason why the solver terminated.
  std::string message = "solve was not called.";

  // Cost of the problem (value of the objective function) before
  // the optimization.
  ResidualInfo initial_cost;

  // Cost of the problem (value of the objective function) after the
  // optimization.
  ResidualInfo final_cost;

  // IterationSummary for each minimizer iteration in order.
  std::vector<IterationSummary> iterations;

  // Number of minimizer iterations in which the step was
  // accepted. Unless use_non_monotonic_steps is true this is also
  // the number of steps in which the objective function value/cost
  // went down.
  int num_successful_steps = -1;

  // Number of minimizer iterations in which the step was rejected
  // either because it did not reduce the cost enough or the step
  // was not numerically valid.
  int num_unsuccessful_steps = -1;

  // All times reported below are wall times.

  // All time spent for expensive logging tasks, that could be disbaled
  // without impacting the solver performance (whereever it's possible to
  // record, i.e. not when logging happens in parallel with other tasks).
  double logging_time_in_seconds = -1.0;

  // Preprocessor time includes everything in the 'optimize' call up until the
  // minimizer loop starts, which  includes allocating the Linearizor
  // (allocating landmark blocks, ...)
  double preprocessor_time_in_seconds = -1.0;

  // Minimizer time is the time spent in the minimizer loop, excluding pre- and
  // post-processing.
  double minimizer_time_in_seconds = -1.0;

  // Postprocessing includes additional computations after the minimizer loop
  // (currently this is always 0).
  double postprocessor_time_in_seconds = -1.0;

  // Some total of all time spent inside the 'optimize' call. This is the sum of
  // preprocessing-, minimizer- and postprocessing-time.
  double total_time_in_seconds = -1.0;

  // Time (in seconds) spent in the linear solver computing the
  // trust region step.
  double linear_solver_time_in_seconds = -1.0;

  // Number of times the Newton step was computed by solving a
  // linear system. This does not include linear solves used by
  // inner iterations.
  int num_linear_solves = -1;

  // Time (in seconds) spent evaluating the residual vector.
  double residual_evaluation_time_in_seconds = -1.0;

  // Number of residual only evaluations.
  int num_residual_evaluations = -1;

  // Time (in seconds) spent evaluating the jacobian matrix.
  double jacobian_evaluation_time_in_seconds = -1.0;

  // Number of Jacobian (and residual) evaluations.
  int num_jacobian_evaluations = -1;

  //  Number of threads specified by the user for Jacobian and
  //  residual evaluation.
  int num_threads_given = -1;

  // Number of threads actually used by the solver for Jacobian and
  // residual evaluation. This number is not equal to
  // num_threads_given if OpenMP is not available.
  int num_threads_used = -1;

  // Number of hardware threads available.
  int num_threads_available = -1;

  // peak resident memory in bytes
  uint64_t resident_memory_peak = 0;
};

}  // namespace rootba
