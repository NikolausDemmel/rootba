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

#include "rootba/solver/linearizor_base.hpp"

#include "rootba/bal/bal_bundle_adjustment_helper.hpp"
#include "rootba/util/time_utils.hpp"

// helper to deal with summary_ and it_summary_ pointers
#define IF_SET(POINTER_VAR) \
  if (POINTER_VAR) POINTER_VAR

namespace rootba {

template <typename Scalar_>
LinearizorBase<Scalar_>::LinearizorBase(
    BalProblem<LinearizorBase::Scalar>& bal_problem,
    const SolverOptions& options, SolverSummary* summary)
    : options_(options), bal_problem_(bal_problem), summary_(summary) {}

template <typename Scalar_>
void LinearizorBase<Scalar_>::start_iteration(IterationSummary* it_summary) {
  it_summary_ = it_summary;
}

template <typename Scalar_>
void LinearizorBase<Scalar_>::compute_error(ResidualInfo& ri) {
  Timer timer;

  BalBundleAdjustmentHelper<Scalar>::compute_error(bal_problem_, options_, ri);

  IF_SET(it_summary_)->residual_evaluation_time_in_seconds += timer.elapsed();
  IF_SET(summary_)->num_residual_evaluations += 1;
}

template <typename Scalar_>
void LinearizorBase<Scalar_>::finish_iteration() {}

template <typename Scalar_>
Scalar_ LinearizorBase<Scalar_>::get_effective_jacobi_scaling_epsilon() {
  if (options_.jacobi_scaling_epsilon > 0) {
    return Scalar(options_.jacobi_scaling_epsilon);
  } else {
    return Sophus::Constants<Scalar>::epsilonSqrt();
  }
}

template <typename Scalar_>
typename ConjugateGradientsSolver<Scalar_>::Summary
LinearizorBase<Scalar_>::pcg(
    const LinearOperator<Scalar>& H_pp, const VecX& b_p,
    std::unique_ptr<Preconditioner<Scalar>>&& preconditioner, VecX& xref) {
  // prepare CG solver options
  typename ConjugateGradientsSolver<Scalar>::Options o;
  typename ConjugateGradientsSolver<Scalar>::PerSolveOptions pso;
  o.min_num_iterations = options_.min_linear_solver_iterations;
  o.max_num_iterations = options_.max_linear_solver_iterations;
  pso.q_tolerance = options_.eta;
  pso.r_tolerance = -1.0;
  pso.preconditioner = std::move(preconditioner);

  // run pcg
  ConjugateGradientsSolver<Scalar> solver(o);
  auto summary = solver.solve(&H_pp, pso, b_p, xref);

  // negate the pose increment, since we solve H(-x) = b
  xref = -xref;

  return summary;
}

#ifdef ROOTBA_INSTANTIATIONS_FLOAT
template class LinearizorBase<float>;
#endif

#ifdef ROOTBA_INSTANTIATIONS_DOUBLE
template class LinearizorBase<double>;
#endif

}  // namespace rootba
