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

#include "rootba/ceres/option_utils.hpp"

#include <iostream>

#include <ceres/loss_function.h>
#include <magic_enum/magic_enum.hpp>

#include "rootba/ceres/loss_function.hpp"
#include "rootba/util/format.hpp"
#include "rootba/util/tbb_utils.hpp"

namespace rootba {

namespace {

template <class Enum, class Enum2>
void set_from_enum_or_die(Enum& output, const Enum2& input,
                          const std::string& option_name) {
  auto str = magic_enum::enum_name(input);
  auto val = magic_enum::enum_cast<Enum>(str);
  CHECK(val.has_value()) << "Could not cast value '{}' of option '{}' to type "
                            "'{}'"_format(str, option_name,
                                          magic_enum::enum_type_name<Enum>());
  output = val.value();
}

}  // namespace

void set_ceres_options(ceres::Solver::Options& out, const SolverOptions& in) {
  out.minimizer_type = ceres::TRUST_REGION;
  out.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  set_from_enum_or_die(out.linear_solver_type, in.linear_solver_type,
                       "linear_solver_type");
  set_from_enum_or_die(out.preconditioner_type, in.preconditioner_type,
                       "preconditioner_type");
  out.use_nonmonotonic_steps = false;
  out.use_inner_iterations = false;
  out.max_num_consecutive_invalid_steps = std::numeric_limits<int>::max();

  out.minimizer_progress_to_stdout = in.verbosity_level > 0;

  // Use effective available hardware threads (respecting process limits, TBB
  // task_arena limits, and TBB global control). We assume that if the user
  // specified a thread limit, this is set as a limit with tbb::global_control
  // outside of this function. The reason to not pass in.num_threads is that the
  // way our custom solvers deal with a specified limit that is larger than the
  // hardware concurrency is that the threads are then still limited by hardware
  // concurrency. To get the same effect for Ceres, we pass the effective
  // concurrency as determined by TBB.
  out.num_threads = tbb_effective_max_concurrency();

  out.max_num_iterations = in.max_num_iterations;

  out.min_relative_decrease = in.min_relative_decrease;

  out.initial_trust_region_radius = in.initial_trust_region_radius;
  out.min_trust_region_radius = in.min_trust_region_radius;
  out.max_trust_region_radius = in.max_trust_region_radius;

  out.min_lm_diagonal = in.min_lm_diagonal;
  out.max_lm_diagonal = in.max_lm_diagonal;

  out.min_linear_solver_iterations = in.min_linear_solver_iterations;
  out.max_linear_solver_iterations = in.max_linear_solver_iterations;
  out.eta = in.eta;

  out.jacobi_scaling = in.jacobi_scaling;
  out.use_explicit_schur_complement = in.use_explicit_schur_complement;

  out.function_tolerance = in.function_tolerance;
  out.gradient_tolerance = in.gradient_tolerance;
  out.parameter_tolerance = in.parameter_tolerance;

  out.check_gradients = in.check_gradients;
  out.gradient_check_relative_precision = in.gradient_check_relative_precision;
  out.gradient_check_numeric_derivative_relative_step_size =
      in.gradient_check_numeric_derivative_relative_step_size;
}

bool ceres_use_projection_validity_check(const SolverOptions& options) {
  switch (options.optimized_cost) {
    case SolverOptions::OptimizedCost::ERROR:
      return false;
    case SolverOptions::OptimizedCost::ERROR_VALID:
      return true;
    default:
      LOG(FATAL) << "optimized_cost option '{}' not implemeted for ceres solver"
                    ""_format(wise_enum::to_string(options.optimized_cost));
  }
}

ceres::LossFunction* ceres_create_loss_function(
    const BalResidualOptions& options) {
  switch (options.robust_norm) {
    case BalResidualOptions::RobustNorm::NONE:
      return nullptr;
    case BalResidualOptions::RobustNorm::HUBER:
      CHECK_GT(options.huber_parameter, 0);
      // return new ceres::HuberLoss(options.huber_parameter);
      return new HuberLossFirstOrderCorrection(options.huber_parameter);
    default:
      LOG(FATAL) << "robust norm {} not implemented"_format(
          wise_enum::to_string(options.robust_norm));
  }
}

void print_ceres_summary(const ceres::Solver::Summary& summary,
                         const SolverOptions& in) {
  if (in.verbosity_level == 0) {
    // nothing
  } else if (in.verbosity_level == 1) {
    // brief report
    std::cout << summary.BriefReport() << "\n";
  } else {
    // >= 2: full report
    std::cout << summary.FullReport() << "\n";
  }
}

}  // namespace rootba
