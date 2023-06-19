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

#include "rootba/bal/ba_log_utils.hpp"

#include <magic_enum/magic_enum.hpp>

namespace rootba {

void log_summary(BaLog::ProblemInfo& log, const DatasetSummary& summary) {
  auto log_stats = [](BaLog::ProblemInfo::Stats& log,
                      const DatasetSummary::Stats& summary) {
    log.mean = summary.mean;
    log.min = summary.min;
    log.max = summary.max;
    log.stddev = summary.stddev;
  };

  log.type = summary.type;
  log.input_path = summary.input_path;
  log.num_cameras = summary.num_cameras;
  log.num_landmarks = summary.num_landmarks;
  log.num_observations = summary.num_observations;
  log.rcs_sparsity = summary.rcs_sparsity;

  log_stats(log.per_lm_obs, summary.per_lm_obs);
  log_stats(log.per_host_lms, summary.per_host_lms);
}

void log_summary(BaLog::PipelineTiming& log,
                 const PipelineTimingSummary& summary) {
  log.load = summary.load_time;
  log.preprocess = summary.preprocess_time;
  log.optimize = summary.optimize_time;
  log.postprocess = summary.postprocess_time;
  log.update_total();
}

void log_summary(BaLog::BaSolver& log, const SolverSummary& summary) {
  log.solver_type = summary.solver_type;
  log.termination_type = summary.termination_type;
  log.message = summary.message;
  log.num_successful_steps = summary.num_successful_steps;
  log.num_unsuccessful_steps = summary.num_unsuccessful_steps;
  log.logging_time_in_seconds = summary.logging_time_in_seconds;
  log.preprocessor_time_in_seconds = summary.preprocessor_time_in_seconds;
  log.minimizer_time_in_seconds = summary.minimizer_time_in_seconds;
  log.postprocessor_time_in_seconds = summary.postprocessor_time_in_seconds;
  log.total_time_in_seconds = summary.total_time_in_seconds;
  log.linear_solver_time_in_seconds = summary.linear_solver_time_in_seconds;
  log.num_linear_solves = summary.num_linear_solves;
  log.residual_evaluation_time_in_seconds =
      summary.residual_evaluation_time_in_seconds;
  log.num_residual_evaluations = summary.num_residual_evaluations;
  log.jacobian_evaluation_time_in_seconds =
      summary.jacobian_evaluation_time_in_seconds;
  log.num_jacobian_evaluations = summary.num_jacobian_evaluations;
  log.num_threads_given = summary.num_threads_given;
  log.num_threads_used = summary.num_threads_used;
  log.num_threads_available = summary.num_threads_available;
  log.resident_memory_peak = summary.resident_memory_peak;
}

void log_summary(BaLog::BaIteration& log, const BaLog::BaIteration* prev_it,
                 const IterationSummary& summary) {
  log.iteration = summary.iteration;
  log.step_is_valid = summary.step_is_valid;
  log.step_is_nonmonotonic = summary.step_is_nonmonotonic;
  log.step_is_successful = summary.step_is_successful;

  if (summary.step_is_successful || (prev_it == nullptr)) {
    // successful iteration: take values from summary
    log.num_obs = summary.cost.all.num_obs;
    log.num_obs_valid = summary.cost.valid.num_obs;
    log.num_obs_valid_change = summary.cost_change.valid.num_obs;
    log.cost = summary.cost.all.error;
    log.cost_change = summary.cost_change.all.error;
    log.cost_valid = summary.cost.valid.error;
    log.cost_valid_change = summary.cost_change.valid.error;
    log.cost_avg_valid = summary.cost.valid.error_avg();
    log.cost_avg_valid_change = summary.cost_change.valid.error_avg;
    log.residual_block_mean = summary.cost.all.residual_mean();
    log.residual_block_valid_mean = summary.cost.valid.residual_mean();
    log.grad_max_norm = summary.gradient_max_norm;
    log.grad_norm = summary.gradient_norm;
    log.step_norm = summary.step_norm;
    log.relative_decrease = summary.relative_decrease;
  } else {
    // unsuccessful iteration: take value from previous iteration for
    // monotonic plots
    log.num_obs = prev_it->num_obs;
    log.num_obs_valid = prev_it->num_obs_valid;
    log.num_obs_valid_change = 0;
    log.cost = prev_it->cost;
    log.cost_change = 0;
    log.cost_valid = prev_it->cost_valid;
    log.cost_valid_change = 0;
    log.cost_avg_valid = prev_it->cost_avg_valid;
    log.cost_avg_valid_change = 0;
    log.residual_block_mean = prev_it->residual_block_mean;
    log.residual_block_valid_mean = prev_it->residual_block_valid_mean;
    log.grad_max_norm = prev_it->grad_max_norm;
    log.grad_norm = prev_it->grad_norm;
    log.step_norm = 0;
    log.relative_decrease = 0;
  }

  log.trust_region_radius = summary.trust_region_radius;
  log.linear_solver_iterations = summary.linear_solver_iterations;
  log.linear_solver_type = summary.linear_solver_type;
  log.iteration_time = summary.iteration_time_in_seconds;
  log.cumulative_time = summary.cumulative_time_in_seconds;
  log.logging_time = summary.logging_time_in_seconds;
  log.step_solver_time = summary.step_solver_time_in_seconds;
  log.residual_evaluation_time = summary.residual_evaluation_time_in_seconds;
  log.jacobian_evaluation_time = summary.jacobian_evaluation_time_in_seconds;
  log.scale_landmark_jacobian_time =
      summary.scale_landmark_jacobian_time_in_seconds;
  log.perform_qr_time = summary.perform_qr_time_in_seconds;
  log.stage1_time = summary.stage1_time_in_seconds;
  log.scale_pose_jacobian_time = summary.scale_pose_jacobian_time_in_seconds;
  log.landmark_damping_time = summary.landmark_damping_time_in_seconds;
  log.compute_preconditioner_time =
      summary.compute_preconditioner_time_in_seconds;
  log.compute_gradient_time = summary.compute_gradient_time_in_seconds;
  log.stage2_time = summary.stage2_time_in_seconds;
  log.prepare_time = summary.prepare_time_in_seconds;
  log.solve_reduced_system_time = summary.solve_reduced_system_time_in_seconds;
  log.back_substitution_time = summary.back_substitution_time_in_seconds;
  log.update_cameras_time = summary.update_cameras_time_in_seconds;
  log.resident_memory = summary.resident_memory;
  log.resident_memory_peak = summary.resident_memory_peak;
}

void log_summary(BaLog& log, const BalPipelineSummary& summary) {
  log.clear();

  log_summary(log.static_data.problem_info, summary.dataset);
  log_summary(log.static_data.timing, summary.timing);
  log_summary(log.static_data.solver, summary.solver);

  // ensure the following emplace_back() never reallocates and the prev_it
  // pointer stays valid
  log.iterations.reserve(summary.solver.iterations.size());
  const BaLog::BaIteration* prev_it = nullptr;
  for (const auto& it : summary.solver.iterations) {
    auto& it_log = log.iterations.emplace_back();
    log_summary(it_log, prev_it, it);
    prev_it = &it_log;  // only valid if emplace_back doesn't realloc
  }
}

}  // namespace rootba
