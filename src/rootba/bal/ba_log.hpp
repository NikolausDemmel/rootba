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

#include <string>
#include <vector>

#include <visit_struct/visit_struct_intrusive.hpp>

#include "rootba/solver/solver_summary.hpp"
#include "rootba/util/eigen_types.hpp"
#include "rootba/util/enum_utils.hpp"

namespace rootba {

struct BaLogOptions;

WISE_ENUM_CLASS(SaveLogFlag, (JSON, 1 << 0), (UBJSON, 1 << 1));
ROOTBA_DECLARE_FLAGS(SaveLogFlags, SaveLogFlag);

struct BaLog {
  /// Static info about the BA problem
  struct ProblemInfo {
    struct Stats {
      BEGIN_VISITABLES(Stats);
      VISITABLE_INIT(double, mean, 0);
      VISITABLE_INIT(double, min, 0);
      VISITABLE_INIT(double, max, 0);
      VISITABLE_INIT(double, stddev, 0);
      END_VISITABLES;
    };

    BEGIN_VISITABLES(ProblemInfo);

    // dataset info
    VISITABLE(std::string, type);  // e.g. "bal"
    VISITABLE(std::string, input_path);

    // map stats
    VISITABLE_INIT(int, num_cameras, 0);
    VISITABLE_INIT(int, num_landmarks, 0);
    VISITABLE_INIT(int, num_observations, 0);
    VISITABLE_INIT(double, rcs_sparsity, 0);

    VISITABLE(Stats, per_lm_obs);
    VISITABLE(Stats, per_host_lms);

    END_VISITABLES;
  };

  /// overall timing (not just BA solver time)
  struct PipelineTiming {
    BEGIN_VISITABLES(PipelineTiming);

    VISITABLE_INIT(double, total, 0);
    VISITABLE_INIT(double, load, 0);
    VISITABLE_INIT(double, preprocess, 0);
    VISITABLE_INIT(double, optimize, 0);
    VISITABLE_INIT(double, postprocess, 0);

    END_VISITABLES;

    void update_total() {
      total = 0;
      total += load;
      total += preprocess;
      total += optimize;
    }
  };

  /// everything solver related that is logged once for the whole solve
  struct BaSolver {
    BEGIN_VISITABLES(BaSolver);

    VISITABLE(std::string, solver_type);

    VISITABLE(TerminationType, termination_type);

    VISITABLE(std::string, message);

    VISITABLE_INIT(int, num_successful_steps, 0);
    VISITABLE_INIT(int, num_unsuccessful_steps, 0);

    VISITABLE_INIT(double, logging_time_in_seconds, 0);
    VISITABLE_INIT(double, preprocessor_time_in_seconds, 0);
    VISITABLE_INIT(double, minimizer_time_in_seconds, 0);
    VISITABLE_INIT(double, postprocessor_time_in_seconds, 0);
    VISITABLE_INIT(double, total_time_in_seconds, 0);
    VISITABLE_INIT(double, linear_solver_time_in_seconds, 0);
    VISITABLE_INIT(int, num_linear_solves, 0);
    VISITABLE_INIT(double, residual_evaluation_time_in_seconds, 0);
    VISITABLE_INIT(int, num_residual_evaluations, 0);
    VISITABLE_INIT(double, jacobian_evaluation_time_in_seconds, 0);
    VISITABLE_INIT(int, num_jacobian_evaluations, 0);

    VISITABLE_INIT(int, num_threads_given, 0);
    VISITABLE_INIT(int, num_threads_used, 0);
    VISITABLE_INIT(int, num_threads_available, 0);

    VISITABLE_INIT(uint64_t, resident_memory_peak, 0);

    END_VISITABLES;
  };

  /// everything that is logged per solver iteration
  struct BaIteration {
    BEGIN_VISITABLES(BaIteration);

    // iteration number (index 0 == initialization)
    VISITABLE_INIT(int, iteration, -1);

    // used solver type
    VISITABLE(std::string, linear_solver_type);

    // success flags
    VISITABLE_INIT(bool, step_is_valid, false);
    VISITABLE_INIT(bool, step_is_nonmonotonic, false);
    VISITABLE_INIT(bool, step_is_successful, false);

    // number of bundle adjustment observations
    VISITABLE_INIT(int, num_obs, 0);
    VISITABLE_INIT(int, num_obs_valid, 0);
    VISITABLE_INIT(int, num_obs_valid_change, 0);

    // maximum likelihood cost of all residuals
    VISITABLE_INIT(double, cost, 0);
    VISITABLE_INIT(double, cost_change, 0);

    // maximum likelihood cost of valid residuals
    VISITABLE_INIT(double, cost_valid, 0);
    VISITABLE_INIT(double, cost_valid_change, 0);

    // avg cost for valid (in bounds) photometric residuals
    VISITABLE_INIT(double, cost_avg_valid, 0);
    VISITABLE_INIT(double, cost_avg_valid_change, 0);

    // gradient 2-norm and max-norm of ml cost gradient (as computed by ceres)
    VISITABLE_INIT(double, grad_projected_norm, 0);
    VISITABLE_INIT(double, grad_projected_max_norm, 0);

    // gradient 2-norm and max-norm of ml cost gradient (in the tangent space)
    VISITABLE_INIT(double, grad_norm, 0);
    VISITABLE_INIT(double, grad_max_norm, 0);

    // mean of 2-norms of residual blocks (i.e. mean reprojection error)
    VISITABLE_INIT(double, residual_block_mean, 0);
    VISITABLE_INIT(double, residual_block_valid_mean, 0);

    // norm of update step
    VISITABLE_INIT(double, step_norm, 0);

    // For trust region algorithms, the ratio of the actual change in
    // cost and the change in the cost of the linearized approximation.
    VISITABLE_INIT(double, relative_decrease, 0);

    // Trust region radius
    VISITABLE_INIT(double, trust_region_radius, 0);

    // Iterative solvers
    VISITABLE_INIT(int, linear_solver_iterations, 0);

    // Time taken to compute one iteration, or cumulative (in seconds)
    VISITABLE_INIT(double, iteration_time, 0);
    VISITABLE_INIT(double, cumulative_time, 0);

    // More timing
    // TODO: make this more fine graned, making sure it is reasonable for both
    // ceres and manual solver; maybe have a struct with all values, and then
    // two members, one for iteration and one for cumulative.

    // time spent on dedicated logging steps
    VISITABLE_INIT(double, logging_time, 0);

    // linear solver
    VISITABLE_INIT(double, step_solver_time, 0);

    // schur solver
    // VISITABLE_INIT(double, time_linearization, 0);
    // VISITABLE_INIT(double, time_linearization_cumulative, 0);
    // VISITABLE_INIT(double, time_schur, 0);
    // VISITABLE_INIT(double, time_schur_cumulative, 0);

    // qr solver
    VISITABLE_INIT(double, residual_evaluation_time, 0);
    VISITABLE_INIT(double, jacobian_evaluation_time, 0);
    VISITABLE_INIT(double, scale_landmark_jacobian_time, 0);
    VISITABLE_INIT(double, perform_qr_time, 0);
    VISITABLE_INIT(double, stage1_time, 0);
    VISITABLE_INIT(double, scale_pose_jacobian_time, 0);
    VISITABLE_INIT(double, landmark_damping_time, 0);
    VISITABLE_INIT(double, compute_preconditioner_time, 0);
    VISITABLE_INIT(double, compute_gradient_time, 0);
    VISITABLE_INIT(double, stage2_time, 0);
    VISITABLE_INIT(double, prepare_time, 0);
    VISITABLE_INIT(double, solve_reduced_system_time, 0);
    VISITABLE_INIT(double, back_substitution_time, 0);
    VISITABLE_INIT(double, update_cameras_time, 0);

    // memory
    VISITABLE_INIT(uint64_t, resident_memory, 0);
    VISITABLE_INIT(uint64_t, resident_memory_peak, 0);

    END_VISITABLES;
  };

  /// Static log data is everything that is not saved per-iteration
  struct Static {
    // WISE_ENUM_CLASS_MEMBER(Status, None, Setup, Running, Failed, Completed);

    BEGIN_VISITABLES(Static);

    // VISITABLE_INIT(Status, status, Status::None);
    VISITABLE(ProblemInfo, problem_info);
    VISITABLE(PipelineTiming, timing);
    VISITABLE(BaSolver, solver);

    END_VISITABLES;
  };

  std::vector<BaIteration> iterations;

  Static static_data;

  void clear();

  /// returns false if there was an error saving the file(s)
  bool save_json(const BaLogOptions& options) const;

  /// returns false if there was an error saving the file(s)
  bool save_json(const std::string& path,
                 SaveLogFlags flags = SaveLogFlag::JSON) const;
};

}  // namespace rootba
