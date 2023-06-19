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
#include "rootba/solver/linearizor_power_sc.hpp"

#include <cmath>
#include <limits>

#include "rootba/cg/conjugate_gradient.hpp"
#include "rootba/cg/preconditioner.hpp"
#include "rootba/sc/linearization_power_sc.hpp"
#include "rootba/util/time_utils.hpp"

// helper to deal with summary_ and it_summary_ pointers
#define IF_SET(POINTER_VAR) \
  if (POINTER_VAR) POINTER_VAR

namespace rootba {

template <class Scalar_>
LinearizorPowerSC<Scalar_>::LinearizorPowerSC(BalProblem<Scalar>& bal_problem,
                                              const SolverOptions& options,
                                              SolverSummary* summary)
    : LinearizorBase<Scalar>(bal_problem, options, summary) {
  // set options
  typename LinearizationPowerSC<Scalar, 9>::Options lsc_options;

  lsc_options.use_householder = options_.use_householder_marginalization;
  lsc_options.use_valid_projections_only =
      options_.use_projection_validity_check();
  lsc_options.jacobi_scaling_eps = Base::get_effective_jacobi_scaling_epsilon();
  lsc_options.residual_options = options_.residual;
  lsc_options.power_order = options_.power_order;

  // create linearization object
  lsc_ = std::make_unique<LinearizationPowerSC<Scalar, 9>>(bal_problem,
                                                           lsc_options);
}

template <class Scalar_>
LinearizorPowerSC<Scalar_>::~LinearizorPowerSC() = default;

template <class Scalar_>
void LinearizorPowerSC<Scalar_>::linearize() {
  // ////////////////////////////////////////////////////////////////////////
  // Stage 1: outside LM solver inner-loop
  // - linearization
  // - scale landmark Jacobians
  // - compute pose Jacobian scale
  // ////////////////////////////////////////////////////////////////////////
  Timer timer_stage1;

  VecX pose_damping_diagonal2;

  Timer timer;
  CHECK(lsc_->linearize_problem())
      << "did not expect numerical failure during linearization";
  IF_SET(it_summary_)->jacobian_evaluation_time_in_seconds = timer.reset();

  pose_damping_diagonal2 = lsc_->get_Jp_diag2();
  lsc_->scale_Jl_cols();
  IF_SET(it_summary_)->scale_landmark_jacobian_time_in_seconds = timer.reset();

  // ceres uses 1.0 / (1.0 + sqrt(SquaredColumnNorm))
  // we use 1.0 / (eps + sqrt(SquaredColumnNorm))
  pose_jacobian_scaling_ = (Base::get_effective_jacobi_scaling_epsilon() +
                            pose_damping_diagonal2.array().sqrt())
                               .inverse();

  IF_SET(it_summary_)->stage1_time_in_seconds = timer_stage1.elapsed();
  IF_SET(summary_)->num_jacobian_evaluations += 1;

  new_linearization_point_ = true;
}

template <class Scalar_>
typename LinearizorPowerSC<Scalar_>::VecX LinearizorPowerSC<Scalar_>::solve(
    Scalar lambda) {
  // ////////////////////////////////////////////////////////////////////////
  // Stage 2: inside LM solver inner-loop
  // - scale pose Jacobians (1st inner it)
  // - dampen
  // ////////////////////////////////////////////////////////////////////////
  Timer timer_stage2;

  // dampen poses
  lsc_->set_pose_damping(lambda);

  Timer timer;

  // scale pose jacobians only on the first inner iteration
  if (new_linearization_point_) {
    lsc_->scale_Jp_cols(pose_jacobian_scaling_);
    IF_SET(it_summary_)->scale_pose_jacobian_time_in_seconds = timer.reset();
  }

  // dampen landmarks
  lsc_->set_landmark_damping(lambda);
  IF_SET(it_summary_)->landmark_damping_time_in_seconds = timer.reset();

  IF_SET(it_summary_)->stage2_time_in_seconds = timer_stage2.elapsed();

  // ////////////////////////////////////////////////////////////////////////
  // Solving:
  // - marginalize landmarks (SC)
  // - solve RCS via power series
  // ////////////////////////////////////////////////////////////////////////

  // prepare the components required for the solver
  VecX b_p;
  {
    Timer timer;
    lsc_->prepare_Hb(b_p);
    IF_SET(it_summary_)->prepare_time_in_seconds = timer.elapsed();
  }

  // run power series solver
  typename ConjugateGradientsSolver<Scalar>::PerSolveOptions pso;
  pso.q_tolerance = options_.eta;
  VecX inc;
  {
    Timer timer;
    const auto summary = lsc_->solve(b_p, inc, pso);
    IF_SET(it_summary_)->solve_reduced_system_time_in_seconds = timer.elapsed();
    IF_SET(it_summary_)->linear_solver_iterations = summary.power_order;
    IF_SET(it_summary_)->linear_solver_message = summary.message;
    IF_SET(it_summary_)->linear_solver_type = "bal_power_sc";
    IF_SET(summary_)->num_linear_solves += 1;
  }

  // if we backtrack, we don't need to rescale jacobians / preconditioners in
  // the next `solve` call
  new_linearization_point_ = false;

  return inc;
}

template <class Scalar_>
Scalar_ LinearizorPowerSC<Scalar_>::apply(VecX&& inc) {
  // backsubstitue landmarks and compute model cost difference
  Timer timer;
  Scalar l_diff = lsc_->back_substitute(inc);
  IF_SET(it_summary_)->back_substitution_time_in_seconds = timer.reset();

  // return directly in case the update is too bad
  if (!std::isfinite(l_diff)) {
    return std::numeric_limits<Scalar>::quiet_NaN();
  }

  // unscale pose increments
  inc.array() *= pose_jacobian_scaling_.array();

  // update cameras
  for (size_t i = 0; i < bal_problem_.cameras().size(); i++) {
    bal_problem_.cameras()[i].apply_inc_pose(inc.template segment<6>(i * 9));
    bal_problem_.cameras()[i].apply_inc_intrinsics(
        inc.template segment<3>(i * 9 + 6));
  }
  IF_SET(it_summary_)->update_cameras_time_in_seconds = timer.elapsed();

  return l_diff;
}

#ifdef ROOTBA_INSTANTIATIONS_FLOAT
template class LinearizorPowerSC<float>;
#endif

#ifdef ROOTBA_INSTANTIATIONS_DOUBLE
template class LinearizorPowerSC<double>;
#endif

}  // namespace rootba
