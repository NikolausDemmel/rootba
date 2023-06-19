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

#include "rootba/ceres/ba_log_utils.hpp"

#include <magic_enum/magic_enum.hpp>

#include "rootba/util/assert.hpp"
#include "rootba/util/system_utils.hpp"
#include "rootba/util/tbb_utils.hpp"

namespace rootba {

void log_ceres_summary(BaLog& log, const std::string& solver_type,
                       const ceres::Solver::Summary& summary) {
  auto& solver_log = log.static_data.solver;

  solver_log.solver_type = solver_type;
  solver_log.termination_type =
      magic_enum::enum_cast<TerminationType>(
          magic_enum::enum_name(summary.termination_type))
          .value();
  solver_log.message = summary.message;
  solver_log.num_successful_steps = summary.num_successful_steps;
  solver_log.num_unsuccessful_steps = summary.num_unsuccessful_steps;
  solver_log.preprocessor_time_in_seconds =
      summary.preprocessor_time_in_seconds;
  solver_log.minimizer_time_in_seconds = summary.minimizer_time_in_seconds;
  solver_log.postprocessor_time_in_seconds =
      summary.postprocessor_time_in_seconds;
  solver_log.total_time_in_seconds = summary.total_time_in_seconds;
  solver_log.linear_solver_time_in_seconds =
      summary.linear_solver_time_in_seconds;
  solver_log.num_linear_solves = summary.num_linear_solves;
  solver_log.residual_evaluation_time_in_seconds =
      summary.residual_evaluation_time_in_seconds;
  solver_log.num_residual_evaluations = summary.num_residual_evaluations;
  solver_log.jacobian_evaluation_time_in_seconds =
      summary.jacobian_evaluation_time_in_seconds;
  solver_log.num_jacobian_evaluations = summary.num_jacobian_evaluations;
  solver_log.num_threads_given = summary.num_threads_given;
  solver_log.num_threads_used = summary.num_threads_used;

  // Effective available hardware threads (respecting process limits).
  solver_log.num_threads_available = tbb_task_arena_max_concurrency();

  {
    MemoryInfo info;
    get_memory_info(info);
    solver_log.resident_memory_peak = info.resident_memory_peak;
  }

  // sum up logging time over all iterations
  solver_log.logging_time_in_seconds = 0;
  for (const auto& it : log.iterations) {
    solver_log.logging_time_in_seconds += it.logging_time;
  }
}

void log_ceres_iteration_summary(
    BaLog::BaIteration& it, const BaLog::BaIteration* prev_it,
    const ceres::IterationSummary& summary,
    const bool ceres_use_projection_validity_check) {
  // copy basic info from ceres
  it.iteration = summary.iteration;
  it.step_is_valid = summary.step_is_valid;
  it.step_is_nonmonotonic = summary.step_is_nonmonotonic;
  it.step_is_successful = summary.step_is_successful;
  if (summary.step_is_successful || (prev_it == nullptr)) {
    // successful iteration: take values from ceres

    // for cost take values from ceres log; if we want more info on
    // error/valid/avg/residual, call log_ceres_error() afterwards
    if (ceres_use_projection_validity_check) {
      it.cost = 0;
      it.cost_change = 0;
      it.cost_valid = summary.cost;
      it.cost_valid_change = summary.cost_change;
    } else {
      it.cost = summary.cost;
      it.cost_change = summary.cost_change;
      it.cost_valid = 0;
      it.cost_valid_change = 0;
    }

    it.grad_projected_norm = summary.gradient_norm;
    it.grad_projected_max_norm = summary.gradient_max_norm;
    it.step_norm = summary.step_norm;
    it.relative_decrease = summary.relative_decrease;

    // values we don't get directly from ceres:
    it.num_obs = 0;
    it.num_obs_valid = 0;
    it.num_obs_valid_change = 0;
    it.cost_avg_valid = 0;
    it.cost_avg_valid_change = 0;
    it.residual_block_mean = 0;
    it.residual_block_valid_mean = 0;
    it.grad_max_norm = 0;
    it.grad_norm = 0;
  } else {
    // unsuccessful iteration: take value from previous iteration for
    // monotonic plots
    it.num_obs = prev_it->num_obs;
    it.num_obs_valid = prev_it->num_obs_valid;
    it.num_obs_valid_change = 0;
    it.cost = prev_it->cost;
    it.cost_change = 0;
    it.cost_valid = prev_it->cost_valid;
    it.cost_valid_change = 0;
    it.cost_avg_valid = prev_it->cost_avg_valid;
    it.cost_avg_valid_change = 0;
    it.residual_block_mean = prev_it->residual_block_mean;
    it.residual_block_valid_mean = prev_it->residual_block_valid_mean;
    it.grad_projected_norm = prev_it->grad_projected_norm;
    it.grad_projected_max_norm = prev_it->grad_projected_max_norm;
    it.grad_max_norm = prev_it->grad_max_norm;
    it.grad_norm = prev_it->grad_norm;
    it.step_norm = 0;
    it.relative_decrease = 0;
  }
  it.trust_region_radius = summary.trust_region_radius;
  it.linear_solver_iterations = summary.linear_solver_iterations;
  it.iteration_time = summary.iteration_time_in_seconds;
  it.step_solver_time = summary.step_solver_time_in_seconds;
  it.cumulative_time = summary.cumulative_time_in_seconds;
}

void log_memory(BaLog::BaIteration& it) {
  MemoryInfo info;
  get_memory_info(info);
  it.resident_memory = info.resident_memory;
  it.resident_memory_peak = info.resident_memory_peak;
}

void log_ceres_error(BaLog::BaIteration& it, const BaLog::BaIteration* prev_it,
                     const ResidualInfo& ri) {
  ROOTBA_ASSERT_MSG(it.iteration >= 0,
                    "need to call log_ceres_summary() before");
  ROOTBA_ASSERT_MSG(
      it.step_is_successful,
      "assuming this was successful iteration (else we reuse prev. cost)");

  it.num_obs = ri.all.num_obs;
  it.num_obs_valid = ri.valid.num_obs;
  it.cost = ri.all.error;
  it.cost_valid = ri.valid.error;
  it.cost_avg_valid = ri.valid.error_avg();
  it.residual_block_mean = ri.all.residual_mean();
  it.residual_block_valid_mean = ri.valid.residual_mean();
  if (it.iteration > 0) {
    // only after initial iteration can be compute a change
    ROOTBA_ASSERT_MSG(prev_it, "if not first iteration, expect prev it log");
    it.num_obs_valid_change = prev_it->num_obs_valid - it.num_obs_valid;
    it.cost_change = prev_it->cost - it.cost;
    it.cost_valid_change = prev_it->cost_valid - it.cost_valid;
    it.cost_avg_valid_change = prev_it->cost_avg_valid - it.cost_avg_valid;
  }
}

}  // namespace rootba
