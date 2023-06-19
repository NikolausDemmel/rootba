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

#include "rootba/cg/preconditioner.hpp"

#include "rootba/bal/bal_problem.hpp"
#include "rootba/sc/linearization_power_sc.hpp"
#include "rootba/sc/linearization_sc.hpp"
#include "rootba/testing/eigen_utils.hpp"
#include "rootba/testing/float_utils.hpp"
#include "rootba/testing/test_types.hpp"

template <typename Scalar_>
class PreconditionerTest : public ::testing::Test {
 public:
  using Scalar = Scalar_;
};

namespace rootba {

TYPED_TEST_SUITE(PreconditionerTest, ScalarTestTypes);

// Test components of PowerSCPreconditioner by comparing the results from
// LinearizationPowerSC solver. Inverting Hpp and preconditioning a vector are
// tested.
TYPED_TEST(PreconditionerTest, PowerSC) {
  using Scalar = typename TestFixture::Scalar;
  using VecX = Vec<Scalar, Eigen::Dynamic>;

  const std::string test_dataset_path =
      "data/rootba/test/bal-ladybug-problem-49-7776-pre-shrink-1800.txt";

  const Scalar damping = 1e-4;
  constexpr int POSE_SIZE = 9;
  const size_t power_order = 20;

  // Ratio of the comparing matrices norm used for calculate the actually
  // threshold. To avoid failing the tests if the comparing matrices have large
  // norm that will naturally result in considerable numerical errors.
  const auto relative_prec = precision_float_or_double<Scalar>(2e-3F, 1e-8);

  // load dataset
  auto bal_problem =
      load_normalized_bal_problem_quiet<Scalar>(test_dataset_path);
  const int num_cams = bal_problem.num_cameras();

  // explicitSC for providing data for the testing preconditioner
  typename LinearizationSC<Scalar, POSE_SIZE>::Options lsc_options;
  LinearizationSC<Scalar, POSE_SIZE> lsc(bal_problem, lsc_options);
  lsc.linearize_problem();
  lsc.set_landmark_damping(damping);
  lsc.scale_Jl_cols();
  lsc.compute_Jp_scale_and_scale_Jp_cols();

  // powerSC
  typename LinearizationPowerSC<Scalar, POSE_SIZE>::Options lpsc_options;
  lpsc_options.power_order = power_order;
  LinearizationPowerSC<Scalar, POSE_SIZE> lpsc(bal_problem, lpsc_options);
  lpsc.linearize_problem();
  lpsc.set_landmark_damping(damping);
  lpsc.scale_Jl_cols();
  lpsc.compute_Jp_scale_and_scale_Jp_cols();

  typename ConjugateGradientsSolver<Scalar>::PerSolveOptions pso{-1, -1,
                                                                 nullptr};

  auto do_test = [&]() {
    // Compute and cache Hll_inv_ in landmark_block
    {
      BlockSparseMatrix<Scalar> H_explicit(num_cams * POSE_SIZE,
                                           num_cams * POSE_SIZE);
      VecX b_p_explicit;
      lsc.get_Hb(H_explicit, b_p_explicit);

      VecX b_p_power;
      lpsc.prepare_Hb(b_p_power);
    }

    const auto Hpp = lsc.get_jacobi();
    PowerSCPreconditioner<Scalar, POSE_SIZE> precond(
        power_order, Hpp.block_diagonal, lsc.get_landmark_blocks(),
        lsc.get_pose_mutex());

    // Compare Hpp^-1 blocks computed by powerSC solver and preconditioner
    for (int i = 0; i < num_cams; ++i) {
      const auto Hpp_inv_power = lpsc.get_Hpp_inv(i);
      const auto Hpp_inv_precond = precond.get_Hpp_inv(i);

      const Scalar prec = Hpp_inv_power.norm() * relative_prec;
      ROOTBA_EXPECT_MAT_NEAR_RELATIVE(Hpp_inv_power, Hpp_inv_precond, prec);
    }

    // Compare the preconditioned result with powerSC solver
    const VecX b = VecX::Random(POSE_SIZE * num_cams);
    VecX x_precond(POSE_SIZE * num_cams);
    VecX x_solver(POSE_SIZE * num_cams);
    precond.solve_assign(b, x_precond);
    lpsc.solve(b, x_solver, pso);

    // A solver solves Hx=-b, but no minus sign for preconditioners
    const Scalar prec = x_solver.norm() * relative_prec;
    ROOTBA_EXPECT_MAT_NEAR_RELATIVE(x_precond, -x_solver, prec);
  };

  // test w/o pose damping (landmark damping is set above)
  do_test();

  // test w/ pose damping
  lsc.set_pose_damping(damping);
  lpsc.set_pose_damping(damping);
  do_test();
}

}  // namespace rootba
