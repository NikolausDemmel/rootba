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

#include "rootba/solver/bal_bundle_adjustment.hpp"

#include <set>
#include <thread>

#include <magic_enum/magic_enum.hpp>

#include "rootba/solver/linearizor.hpp"
#include "rootba/solver/solver_summary.hpp"
#include "rootba/util/format.hpp"
#include "rootba/util/own_or_reference.hpp"
#include "rootba/util/system_utils.hpp"
#include "rootba/util/tbb_utils.hpp"
#include "rootba/util/time_utils.hpp"

namespace rootba {

namespace {

void finish_iteration(SolverSummary& summary, IterationSummary& it_summary) {
  // step_solver_time: trying to count like ceres, all linear solver steps;
  // note that in "staged" execution mode 'scale_landmark_jacobian_time' and
  // 'perform_qr_time_in_seconds' will be 0 and thus not be reflected in the
  // step_solver_time (otoh we also cannot count the whole stage1_time, since
  // that would include jacobian_evaluation_time).
  it_summary.step_solver_time_in_seconds =
      it_summary.scale_landmark_jacobian_time_in_seconds +
      it_summary.perform_qr_time_in_seconds +
      it_summary.stage2_time_in_seconds +
      it_summary.solve_reduced_system_time_in_seconds +
      it_summary.back_substitution_time_in_seconds;

  // cost change
  if (it_summary.iteration > 0) {
    it_summary.cost_change =
        it_summary.cost.compared_to(summary.iterations.back().cost);
  }

  // memory
  {
    MemoryInfo mi;
    if (get_memory_info(mi)) {
      it_summary.resident_memory = mi.resident_memory;
      it_summary.resident_memory_peak = mi.resident_memory_peak;
    }
  }

  // push iteration
  summary.iterations.push_back(it_summary);

  // flush output
  std::cout.flush();
}

void finish_solve(SolverSummary& summary, const SolverOptions& options) {
  switch (options.solver_type) {
    case SolverOptions::SolverType::SQUARE_ROOT:
      summary.solver_type = "bal_qr";
      break;
    case SolverOptions::SolverType::SCHUR_COMPLEMENT:
      summary.solver_type = "bal_sc";
      break;
    default:
      LOG(FATAL) << "unreachable";
  }

  summary.initial_cost = summary.iterations.front().cost;

  // final_cost: find last successful iteration
  for (auto it = summary.iterations.rbegin(); it != summary.iterations.rend();
       ++it) {
    if (it->step_is_successful) {
      summary.final_cost = it->cost;
      break;
    }
  }

  // TODO: this check depends on when error we use for LM... so maybe remove it
  // CHECK_LE(summary.final_cost.valid.error, summary.initial_cost.valid.error);

  summary.num_successful_steps =
      -1;  // don't count it 0, for which step_is_successful == true
  summary.num_unsuccessful_steps = 0;
  for (const auto& it : summary.iterations) {
    if (it.step_is_successful) {
      ++summary.num_successful_steps;
    } else {
      ++summary.num_unsuccessful_steps;
    }
  }

  summary.logging_time_in_seconds = 0;
  summary.preprocessor_time_in_seconds = 0;
  summary.postprocessor_time_in_seconds = 0;
  summary.total_time_in_seconds = summary.minimizer_time_in_seconds;

  summary.linear_solver_time_in_seconds = 0;
  summary.residual_evaluation_time_in_seconds = 0;
  summary.jacobian_evaluation_time_in_seconds = 0;
  for (const auto& it : summary.iterations) {
    summary.linear_solver_time_in_seconds += it.step_solver_time_in_seconds;
    summary.residual_evaluation_time_in_seconds +=
        it.residual_evaluation_time_in_seconds;
    summary.jacobian_evaluation_time_in_seconds +=
        it.jacobian_evaluation_time_in_seconds;
  }

  // memory & threads
  {
    MemoryInfo mi;
    if (get_memory_info(mi)) {
      summary.resident_memory_peak = mi.resident_memory_peak;
    }
  }

  // FIXME: this doesn't respect cgroup cpusets like on slurm
  // (TbbConcurrencyObserver does seem to work as expected though, and create
  // only as many threads as the user can use concurrently). --> check all uses
  // of hardware_concurrency() in the code base
  summary.num_threads_available = std::thread::hardware_concurrency();
}

// compute actual cost difference for deciding if LM step was successful; this
// will be compared to model cost change
double compute_cost_decrease(
    const ResidualInfo& ri_before, const ResidualInfo& ri_after,
    const SolverOptions::OptimizedCost& optimized_cost) {
  switch (optimized_cost) {
    case SolverOptions::OptimizedCost::ERROR:
      return ri_before.all.error - ri_after.all.error;
    case SolverOptions::OptimizedCost::ERROR_VALID:
      return ri_before.valid.error - ri_after.valid.error;
    case SolverOptions::OptimizedCost::ERROR_VALID_AVG:
      return ri_before.valid.error_avg() - ri_after.valid.error_avg();
    default:
      LOG(FATAL) << "unreachable";
  }
}

// check termination based on change in cost value
bool function_tolerance_reached(const IterationSummary& it,
                                const SolverOptions& options,
                                std::string& message) {
  double cost;
  double change;
  switch (options.optimized_cost) {
    case SolverOptions::OptimizedCost::ERROR:
      cost = it.cost.all.error;
      change = std::abs(it.cost_change.all.error);
      break;
    case SolverOptions::OptimizedCost::ERROR_VALID:
    case SolverOptions::OptimizedCost::ERROR_VALID_AVG:
      cost = it.cost.valid.error;
      change = std::abs(it.cost_change.valid.error);
      break;
    default:
      LOG(FATAL) << "unreachable";
  }

  if (change <= options.function_tolerance * cost) {
    message =
        "Function tolerance reached. |cost_change|/cost: {} <= {}"
        ""_format(change / cost, options.function_tolerance);
    return true;
  } else {
    return false;
  }
}

// format info about new error, depending on 'optimized_cost' config
std::string format_new_error_info(
    const ResidualInfo& ri,
    const SolverOptions::OptimizedCost& optimized_cost) {
  switch (optimized_cost) {
    case SolverOptions::OptimizedCost::ERROR:
      return "error: {:.4e} (mean res: {:.2f}, num valid: {})"
             ""_format(ri.all.error, ri.all.residual_mean(), ri.valid.num_obs);
    case SolverOptions::OptimizedCost::ERROR_VALID:
      return "error valid: {:.4e} (mean res: {:.2f}, num: {})"
             ""_format(ri.valid.error, ri.valid.residual_mean(),
                       ri.valid.num_obs);
    case SolverOptions::OptimizedCost::ERROR_VALID_AVG:
      return "error valid avg: {:.4e} (mean res: {:.2f}, num: {})"
             ""_format(ri.valid.error_avg(), ri.valid.residual_mean(),
                       ri.valid.num_obs);
    default:
      LOG(FATAL) << "unreachable";
  }
}

void check_options(const SolverOptions& options) {
  CHECK(options.min_trust_region_radius <= options.initial_trust_region_radius)
      << "Invalid configuration";
  CHECK(options.initial_trust_region_radius <= options.max_trust_region_radius)
      << "Invalid configuration";

  if (options.preconditioner_type !=
          SolverOptions::PreconditionerType::JACOBI &&
      options.preconditioner_type !=
          SolverOptions::PreconditionerType::SCHUR_JACOBI) {
    LOG(FATAL) << "predonditioner {} not implemented"_format(
        wise_enum::to_string(options.preconditioner_type));
  }

  if (options.residual.robust_norm != BalResidualOptions::RobustNorm::NONE &&
      options.residual.robust_norm != BalResidualOptions::RobustNorm::HUBER) {
    LOG(FATAL) << "robust norm {} not implemented"_format(
        wise_enum::to_string(options.residual.robust_norm));
  }

  CHECK_GE(options.jacobi_scaling_epsilon, 0);
}

template <typename Scalar>
void optimize_lm_ours(BalProblem<Scalar>& bal_problem,
                      const SolverOptions& solver_options,
                      SolverSummary& summary) {
  ScopedTbbThreadLimit thread_limit(solver_options.num_threads);
  TbbConcurrencyObserver concurrency_observer;

  Timer timer_total;

  using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  const Scalar min_lambda(1.0 / solver_options.max_trust_region_radius);
  const Scalar max_lambda(1.0 / solver_options.min_trust_region_radius);
  const Scalar vee_factor(solver_options.vee_factor);
  const Scalar initial_vee(solver_options.initial_vee);

  const int max_lm_iter = solver_options.max_num_iterations;

  Scalar lambda(1.0 / solver_options.initial_trust_region_radius);
  Scalar lambda_vee(initial_vee);

  // check options are valid and abort otherwise
  check_options(solver_options);

  summary = SolverSummary();
  summary.num_linear_solves = 0;
  summary.num_residual_evaluations = 0;
  summary.num_jacobian_evaluations = 0;

  std::unique_ptr<Linearizor<Scalar>> linearizor =
      Linearizor<Scalar>::create(bal_problem, solver_options, &summary);

  bool terminated = false;

  for (int it = 0; it <= max_lm_iter && !terminated && !bal_problem.terminate();) {
    IterationSummary it_summary;
    it_summary.iteration = it;
    linearizor->start_iteration(&it_summary);

    Timer timer_iteration;

    // TODO: avoid recomputation of error if we call linearizor->linearize()
    // anyway (only for iteration 0 we can call compute_error instead)...
    ResidualInfo ri;
    linearizor->compute_error(ri);

    // std::cout << "Iteration {}, {}\n"_format(
    //     it, error_summary_oneline(
    //             ri, solver_options.use_projection_validity_check()));

    CHECK(ri.is_numerically_valid)
        << "did not expect numerical failure during linearization";

    // iteration 0 is just error evaluation and logging
    if (it == 0) {
      linearizor->finish_iteration();
      it_summary.cost = ri;
      it_summary.trust_region_radius = 1 / lambda;
      it_summary.iteration_time_in_seconds = timer_iteration.elapsed();
      it_summary.cumulative_time_in_seconds = timer_total.elapsed();
      it_summary.step_is_successful = true;
      it_summary.step_is_valid = true;
      finish_iteration(summary, it_summary);
      ++it;
      continue;
    }

    linearizor->linearize();

    // std::cout << "\t[INFO] Stage 1 time {:.3f}s.\n"_format(
    //     it_summary.stage1_time_in_seconds);

    // Don't limit inner lm iterations (to be in line with ceres)
    constexpr int MAX_INNER_IT = std::numeric_limits<int>::max();

    for (int j = 0; j < MAX_INNER_IT && it <= max_lm_iter && !terminated && !bal_problem.terminate(); j++) {
      if (j > 0) {
        // std::cout << "Iteration {}, backtracking\n"_format(it);

        it_summary = IterationSummary();
        it_summary.iteration = it;
        linearizor->start_iteration(&it_summary);

        timer_iteration.reset();
      }

      // dampen and solve linear system
      VecX inc = linearizor->solve(lambda);

      // std::cout << "\t[INFO] Stage 2 time {:.3f}s.\n"_format(
      //     it_summary.stage2_time_in_seconds);

      // std::cout << "\t[CG] Summary: {} Time {:.3f}s. Time per iteration "
      //              "{:.3f}s\n"_format(
      //                  it_summary.linear_solver_message,
      //                  it_summary.solve_reduced_system_time_in_seconds,
      //                  it_summary.solve_reduced_system_time_in_seconds /
      //                      it_summary.linear_solver_iterations);

      // TODO: cleanly abort linear solver on numerical issue

      // TODO: add to log: gradient norm, step norm

      if (!inc.array().isFinite().all()) {
        it_summary.step_is_valid = false;
        it_summary.step_is_successful = false;

        const double iteration_time = timer_iteration.elapsed();
        const double cumulative_time = timer_total.elapsed();

        std::string reason = it_summary.step_is_valid ? "Reject" : "Invalid";

        // std::cout
        //     << "\t[{}] {}, lambda: {:.1e}, cg_iter: {}, it_time: "
        //        "{:.3f}s, total_time: {:.3f}s\n"
        //        ""_format(
        //            reason,
        //            "Numeric issues when computing increment (contains NaNs)",
        //            lambda, it_summary.linear_solver_iterations, iteration_time,
        //            cumulative_time);

        lambda = lambda_vee * lambda;
        lambda_vee *= vee_factor;

        linearizor->finish_iteration();
        it_summary.trust_region_radius = 1 / lambda;
        it_summary.iteration_time_in_seconds = iteration_time;
        it_summary.cumulative_time_in_seconds = cumulative_time;
        it_summary.step_is_successful = false;
        finish_iteration(summary, it_summary);

        it++;

        if (lambda > max_lambda) {
          terminated = true;
          summary.termination_type = NO_CONVERGENCE;
          summary.message =
              "Solver did not converge and reached maximum "
              "damping lambda of {}"_format(max_lambda);
        }

        continue;
      }

      bal_problem.backup();

      Scalar l_diff = linearizor->apply(std::move(inc));

      ResidualInfo ri2;
      linearizor->compute_error(ri2);
      it_summary.cost = ri2;

      if (!ri2.is_numerically_valid) {
        it_summary.step_is_valid = false;
        it_summary.step_is_successful = false;
        // std::cout << "\t[EVAL] failed to evaluate cost: {}"_format(
        //     error_summary_oneline(
        //         ri2, solver_options.use_projection_validity_check()));
      } else {
        // compute "ri - ri2", depending on 'optimized_cost' config
        Scalar f_diff(
            compute_cost_decrease(ri, ri2, solver_options.optimized_cost));

        // ... only in case of ERROR_VALID_AVG, do we need to normalize the
        // model cost by the number of observations. The model assumes the
        // number of valid residuals remains unchanged, so we simply divide the
        // difference by the number of observations before the update.
        if (solver_options.optimized_cost ==
            SolverOptions::OptimizedCost::ERROR_VALID_AVG) {
          l_diff /= ri.valid.num_obs;
        }

        Scalar step_quality = f_diff / l_diff;

        // std::cout << "\t[EVAL] f_diff {:.4e} l_diff {:.4e} "
        //              "step_quality {:.4e} ri1 {:.4e} ri2 {:.4e}\n"
        //              ""_format(f_diff, l_diff, step_quality, ri.valid.error,
        //                        ri2.valid.error);

        it_summary.relative_decrease = step_quality;

        it_summary.step_is_valid = l_diff > 0;
        it_summary.step_is_successful =
            it_summary.step_is_valid &&
            step_quality > solver_options.min_relative_decrease;
      }

      if (it_summary.step_is_successful) {
        ROOTBA_ASSERT(it_summary.step_is_valid);

        const double iteration_time = timer_iteration.elapsed();
        const double cumulative_time = timer_total.elapsed();

        // std::cout << "\t[Success] {}, lambda: {:.1e}, cg_iter: {}, it_time: "
        //              "{:.3f}s, total_time: {:.3f}s\n"
        //              ""_format(format_new_error_info(
        //                            ri2, solver_options.optimized_cost),
        //                        lambda, it_summary.linear_solver_iterations,
        //                        iteration_time, cumulative_time);

        lambda *= Scalar(std::max(
            1.0 / 3, 1 - std::pow(2 * it_summary.relative_decrease - 1, 3)));
        lambda = std::max(min_lambda, lambda);

        lambda_vee = initial_vee;

        linearizor->finish_iteration();
        it_summary.trust_region_radius = 1 / lambda;
        it_summary.iteration_time_in_seconds = iteration_time;
        it_summary.cumulative_time_in_seconds = cumulative_time;
        finish_iteration(summary, it_summary);

        it++;

        // check function tolerance
        if (function_tolerance_reached(summary.iterations.back(),
                                       solver_options, summary.message)) {
          terminated = true;
          summary.termination_type = CONVERGENCE;
        }

        // stop inner lm loop
        break;
      } else {
        const double iteration_time = timer_iteration.elapsed();
        const double cumulative_time = timer_total.elapsed();

        std::string reason = it_summary.step_is_valid ? "Reject" : "Invalid";

        // std::cout << "\t[{}] {}, lambda: {:.1e}, cg_iter: {}, it_time: "
        //              "{:.3f}s, total_time: {:.3f}s\n"
        //              ""_format(reason,
        //                        format_new_error_info(
        //                            ri2, solver_options.optimized_cost),
        //                        lambda, it_summary.linear_solver_iterations,
        //                        iteration_time, cumulative_time);

        lambda = lambda_vee * lambda;
        lambda_vee *= vee_factor;

        linearizor->finish_iteration();
        it_summary.trust_region_radius = 1 / lambda;
        it_summary.iteration_time_in_seconds = iteration_time;
        it_summary.cumulative_time_in_seconds = cumulative_time;
        it_summary.step_is_successful = false;
        finish_iteration(summary, it_summary);

        bal_problem.restore();
        it++;

        if (lambda > max_lambda) {
          terminated = true;
          summary.termination_type = NO_CONVERGENCE;
          summary.message =
              "Solver did not converge and reached maximum "
              "damping lambda of {}"_format(max_lambda);
        }
      }
    }
  }

  if (!terminated) {
    summary.termination_type = NO_CONVERGENCE;
    summary.message =
        "Solver did not converge after maximum number of "
        "{} iterations"_format(max_lm_iter);
  }

  summary.minimizer_time_in_seconds = timer_total.elapsed();

  summary.num_threads_given = solver_options.num_threads;
  summary.num_threads_used = concurrency_observer.get_peak_concurrency();

  finish_solve(summary, solver_options);

  // std::cout << "Final Cost: {}\n"_format(error_summary_oneline(
  //     summary.final_cost, solver_options.use_projection_validity_check()));
  // std::cout << "{}: {}\n"_format(
  //     magic_enum::enum_name(summary.termination_type), summary.message);
  const double opt_time = timer_total.elapsed();
  std::cout << "total time is:" << opt_time*1000 << std::endl;
  std::cout.flush();
}

}  // namespace

template <typename Scalar>
void bundle_adjust_manual(BalProblem<Scalar>& bal_problem,
                          const SolverOptions& solver_options,
                          SolverSummary* output_solver_summary,
                          PipelineTimingSummary* output_timing_summary) {
  OwnOrReference<SolverSummary> solver_summary(output_solver_summary);
  optimize_lm_ours(bal_problem, solver_options, *solver_summary);

  if (output_timing_summary) {
    output_timing_summary->optimize_time =
        solver_summary->total_time_in_seconds;
  }

  // LOG(INFO) << solver_summary->message;

  // TODO: Print summary similar to ceres: oneline, or full
}

#ifdef ROOTBA_INSTANTIATIONS_FLOAT
template void bundle_adjust_manual(
    BalProblem<float>& bal_problem, const SolverOptions& solver_options,
    SolverSummary* output_solver_summary,
    PipelineTimingSummary* output_timing_summary);
#endif

#ifdef ROOTBA_INSTANTIATIONS_DOUBLE
template void bundle_adjust_manual(
    BalProblem<double>& bal_problem, const SolverOptions& solver_options,
    SolverSummary* output_solver_summary,
    PipelineTimingSummary* output_timing_summary);
#endif

}  // namespace rootba
