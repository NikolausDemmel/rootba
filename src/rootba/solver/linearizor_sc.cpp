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
#include "rootba/solver/linearizor_sc.hpp"

#include "rootba/cg/conjugate_gradient.hpp"
#include "rootba/cg/preconditioner.hpp"
#include "rootba/sc/linearization_sc.hpp"
#include "rootba/util/time_utils.hpp"

// helper to deal with summary_ and it_summary_ pointers
#define IF_SET(POINTER_VAR) \
  if (POINTER_VAR) POINTER_VAR

namespace rootba {

template <class Scalar_>
LinearizorSC<Scalar_>::LinearizorSC(BalProblem<Scalar>& bal_problem,
                                    const SolverOptions& options,
                                    SolverSummary* summary)
    : LinearizorBase<Scalar>(bal_problem, options, summary) {
  // set options
  typename LinearizationSC<Scalar, 9>::Options lsc_options;

  lsc_options.use_householder = options_.use_householder_marginalization;
  lsc_options.use_valid_projections_only =
      options_.use_projection_validity_check();
  lsc_options.jacobi_scaling_eps = Base::get_effective_jacobi_scaling_epsilon();
  lsc_options.residual_options = options_.residual;

  // create linearization object
  lsc_ = std::make_unique<LinearizationSC<Scalar, 9>>(bal_problem, lsc_options);
}

template <class Scalar_>
LinearizorSC<Scalar_>::~LinearizorSC() = default;

template <class Scalar_>
void LinearizorSC<Scalar_>::linearize() {
  // ////////////////////////////////////////////////////////////////////////
  // Stage 1: outside LM solver inner-loop
  // - linearization
  // - scale landmark Jacobians
  // - compute pose Jacobian scale
  // ////////////////////////////////////////////////////////////////////////
  Timer timer_stage1;

  VecX pose_damping_diagonal2;

  // TODO: would a staged version make sense here (consider stage 1 and 2)?

  Timer timer;
  CHECK(lsc_->linearize_problem())
      << "did not expect numerical failure during linearization";
  IF_SET(it_summary_)->jacobian_evaluation_time_in_seconds = timer.reset();

  pose_damping_diagonal2 = lsc_->get_Jp_diag2();
  lsc_->scale_Jl_cols();
  IF_SET(it_summary_)->scale_landmark_jacobian_time_in_seconds = timer.reset();

  // TODO: maybe we should reset pose_jacobian_scaling_ at iteration end to
  // avoid accidental use of outdated info?

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
typename LinearizorSC<Scalar_>::VecX LinearizorSC<Scalar_>::solve(
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

  // scale landmark jacobians only on the first inner iteration
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
  // - invert preconditioner
  // - run PCG
  // ////////////////////////////////////////////////////////////////////////

  // TODO: move Schur complement part to stage2 above? And also add to timing
  // log as "compute_schure_complement_time" or similar?

  // compute Schur complement
  const int num_cams = bal_problem_.num_cameras();
  const int pose_size = lsc_->POSE_SIZE;
  BlockSparseMatrix<Scalar> H_pp(num_cams * pose_size, num_cams * pose_size);
  VecX b_p;
  lsc_->get_Hb(H_pp, b_p);
  H_pp.recompute_keys();

  // create and invert the proconditioner
  CHECK(options_.preconditioner_type ==
        SolverOptions::PreconditionerType::SCHUR_JACOBI)
      << "not implemented";
  std::unique_ptr<Preconditioner<Scalar>> precond;
  {
    Timer timer;
    precond.reset(new BlockDiagonalPreconditioner<Scalar>(
        num_cams, pose_size, H_pp.block_storage, nullptr));
    IF_SET(it_summary_)->compute_preconditioner_time_in_seconds = timer.reset();
  }

  // run pcg
  VecX inc = VecX::Zero(lsc_->num_cols_reduced());
  {
    Timer timer;
    typename ConjugateGradientsSolver<Scalar>::Summary cg_summary =
        Base::pcg(H_pp, b_p, std::move(precond), inc);
    IF_SET(it_summary_)->solve_reduced_system_time_in_seconds = timer.elapsed();
    IF_SET(it_summary_)->linear_solver_message = cg_summary.message;
    IF_SET(it_summary_)->linear_solver_iterations = cg_summary.num_iterations;
    IF_SET(summary_)->num_linear_solves += 1;
  }

  // if we backtrack, we don't need to rescale jacobians / preconditioners in
  // the next `solve` call
  new_linearization_point_ = false;

  return inc;
}

template <class Scalar_>
Scalar_ LinearizorSC<Scalar_>::apply(VecX&& inc) {
  // backsubstitue landmarks and compute model cost difference
  Timer timer;
  Scalar l_diff = lsc_->back_substitute(inc);
  IF_SET(it_summary_)->back_substitution_time_in_seconds = timer.elapsed();

  // unscale pose increments
  inc.array() *= pose_jacobian_scaling_.array();

  // update cameras
  for (size_t i = 0; i < bal_problem_.cameras().size(); i++) {
    bal_problem_.cameras()[i].apply_inc_pose(inc.template segment<6>(i * 9));
    bal_problem_.cameras()[i].apply_inc_intrinsics(
        inc.template segment<3>(i * 9 + 6));
  }

  return l_diff;
}

#ifdef ROOTBA_INSTANTIATIONS_FLOAT
template class LinearizorSC<float>;
#endif

#ifdef ROOTBA_INSTANTIATIONS_DOUBLE
template class LinearizorSC<double>;
#endif

}  // namespace rootba
