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

#include "rootba/ceres/bal_iteration_callback.hpp"

#include "rootba/bal/bal_bundle_adjustment_helper.hpp"
#include "rootba/ceres/ba_log_utils.hpp"
#include "rootba/ceres/option_utils.hpp"
#include "rootba/util/time_utils.hpp"

namespace rootba {

BalIterationCallback::BalIterationCallback(BaLog& log,
                                           BalProblem<double>& bal_problem,
                                           const VecXd& camera_state,
                                           const SolverOptions& options)
    : log_(log),
      bal_problem_(bal_problem),
      camera_state_(camera_state),
      options_(options) {}

ceres::CallbackReturnType BalIterationCallback::operator()(
    const ceres::IterationSummary& summary) {
  if (options_.log.disable_all) {
    return ceres::SOLVER_CONTINUE;
  }

  Timer timer;

  ROOTBA_ASSERT(summary.iteration == 0 || !log_.iterations.empty());
  auto& it = log_.iterations.emplace_back();  // may reallocate
  const BaLog::BaIteration* prev_it =
      summary.iteration > 0 ? &log_.iterations.at(log_.iterations.size() - 2)
                            : nullptr;

  // ceres summary
  log_ceres_iteration_summary(it, prev_it, summary,
                              ceres_use_projection_validity_check(options_));

  // memory
  log_memory(it);

  // compute (valid) error if step was successful and state changed
  if (it.step_is_successful) {
    bal_problem_.copy_from_camera_state(camera_state_);

    ResidualInfo ri;
    BalBundleAdjustmentHelper<double>::compute_error(bal_problem_, options_,
                                                     ri);
    log_ceres_error(it, prev_it, ri);

    // TODO: compare ceres reported error and error in ri and warn/error if
    // there is mismatch
  }

  // record time taken for the whole callback
  it.logging_time = timer.elapsed();

  return ceres::SOLVER_CONTINUE;
}

}  // namespace rootba
