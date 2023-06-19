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

#include "rootba/qr/linearization_qr.hpp"

#include "rootba/bal/bal_problem.hpp"
#include "rootba/qr/linearization_utils.hpp"
#include "rootba/sc/linearization_sc.hpp"
#include "rootba/testing/eigen_utils.hpp"
#include "rootba/testing/float_utils.hpp"
#include "rootba/testing/test_types.hpp"

namespace rootba {

#if defined(ROOTBA_INSTANTIATIONS_FLOAT) || \
    defined(ROOTBA_INSTANTIATIONS_DOUBLE)

template <typename Scalar_>
class LinearizationQRTest : public ::testing::Test {
 public:
  using Scalar = Scalar_;
};

constexpr int POSE_SIZE = 9;
const std::string TEST_DATASET_PATH =
    "data/rootba/test/bal-ladybug-problem-49-7776-pre-shrink-1800.txt";

TYPED_TEST_SUITE(LinearizationQRTest, ScalarTestTypes);

TYPED_TEST(LinearizationQRTest, BasicLinearAlgebraTest) {
  using Scalar = typename TestFixture::Scalar;
  using VecX = Vec<Scalar, Eigen::Dynamic>;

  const std::string test_dataset_path =
      "data/rootba/test/bal-ladybug-problem-49-7776-pre-shrink-1800.txt";

  // load dataset
  auto bal_problem =
      load_normalized_bal_problem_quiet<Scalar>(test_dataset_path);

  typename LinearizationQR<Scalar, 9>::Options lqr_options;
  LinearizationQR<Scalar, 9> lqr(bal_problem, lqr_options);
  lqr.linearize_problem();
  lqr.perform_qr();

  auto sm1 = lqr.get_Q2TJp();

  {
    VecX rand1;
    rand1.setRandom(lqr.num_cols());

    VecX r1_1 = sm1 * rand1;
    VecX r1_2 = lqr.get_Q2TJp_postmult_x(rand1);

    EXPECT_TRUE(r1_1.isApprox(r1_2));
  }

  {
    VecX rand2;
    rand2.setRandom(lqr.num_rows_reduced());

    VecX r2_1 = sm1.transpose() * rand2;
    VecX r2_2 = lqr.get_Q2TJp_premult_x(rand2);

    EXPECT_TRUE(r2_1.isApprox(r2_2));
  }

  {
    VecX rand3;
    rand3.setRandom(lqr.num_cols());

    VecX r3_1 = sm1.transpose() * (sm1 * rand3);
    VecX r3_2 = lqr.get_Q2TJp_T_Q2TJp_mult_x(rand3);

    EXPECT_TRUE(r3_1.isApprox(r3_2));
  }
}

// Test that get_stage1, get_stage2, right_multiply, back_substitute for
// LinearizationQR and LinearizationSC gives the same result:
//  - pose jacobian norms (for jacobi scaling)
//  - RCS gradient
//  - SCHUR_JACOBI preconditioner
//  - H*x right multiplication for RCS
//  - model cost difference
//  - landmark state update
TYPED_TEST(LinearizationQRTest, EquivalenceTest) {
  using Scalar = typename TestFixture::Scalar;
  using VecX = Vec<Scalar, Eigen::Dynamic>;

  // expected accuracy of test depending on floating point precision
  const auto prec = default_test_precision<Scalar>();

  // not too small damping to see the effect of errors in implementation
  const Scalar lambda = 1e-1;

  // test with default jacobian scaling epsilon
  const auto jacobi_scaling_eps = default_testing_jacobi_scaling_eps<Scalar>();

  // load dataset and make second copy
  auto bal_problem_qr =
      load_normalized_bal_problem_quiet<Scalar>(TEST_DATASET_PATH);
  auto bal_problem_sc = bal_problem_qr;

  const size_t num_cams = bal_problem_qr.cameras().size();

  // qr
  typename LinearizationQR<Scalar, POSE_SIZE>::Options lqr_options;
  lqr_options.lb_options.jacobi_scaling_eps = jacobi_scaling_eps;
  LinearizationQR<Scalar, POSE_SIZE> lqr(bal_problem_qr, lqr_options);

  // sc
  typename LinearizationSC<Scalar, POSE_SIZE>::Options lsc_options;
  lsc_options.jacobi_scaling_eps = jacobi_scaling_eps;
  LinearizationSC<Scalar, POSE_SIZE> lsc(bal_problem_sc, lsc_options);

  auto test_equivalent = [&]() {
    // stage 1 qr
    const VecX pose_jacobian_scale2_qr = lqr.get_stage1(nullptr);
    const VecX pose_jacobian_scaling_qr =
        compute_jacobi_scaling(pose_jacobian_scale2_qr, jacobi_scaling_eps);

    // stage 1 sc
    lsc.linearize_problem();
    lsc.scale_Jl_cols();

    // stage 2 qr
    VecX b_qr;
    IndexedBlocks<Scalar> precond_blocks_qr;
    lqr.get_stage2(lambda, &pose_jacobian_scaling_qr, &precond_blocks_qr, b_qr);

    // stage 2 sc
    lsc.compute_Jp_scale_and_scale_Jp_cols();
    lsc.set_landmark_damping(lambda);
    BlockSparseMatrix<Scalar> H_pp_sc(num_cams * POSE_SIZE,
                                      num_cams * POSE_SIZE);
    VecX b_sc;
    lsc.get_Hb(H_pp_sc, b_sc);

    // test b
    ROOTBA_EXPECT_MAT_NEAR_RELATIVE(b_qr, b_sc, prec);

    // test preconditioner
    for (const auto& [idx, block_qr] : precond_blocks_qr) {
      ASSERT_TRUE(H_pp_sc.block_storage.count(idx));
      ROOTBA_EXPECT_MAT_NEAR_RELATIVE(block_qr, H_pp_sc.block_storage[idx],
                                      prec);
    }

    // we want to test right_multiply, so just generate a random residual vector
    VecX x_pose;
    x_pose.setRandom(num_cams * POSE_SIZE);

    const VecX res_qr = lqr.right_multiply(x_pose);
    const VecX res_sc = H_pp_sc.right_multiply(x_pose);

    // test right_multiply result
    ROOTBA_EXPECT_MAT_NEAR_RELATIVE(res_qr, res_sc, prec);

    // We want to test back_substitute, so just generate a random pose
    // increment (should be small to avoid too large rotations).
    const VecX pose_inc = VecX::Random(num_cams * POSE_SIZE) * 0.01;

    // back substitution
    const Scalar l_diff_qr = lqr.back_substitute(pose_inc);
    const Scalar l_diff_sc = lsc.back_substitute(pose_inc);

    // test l_diff
    ROOTBA_EXPECT_NEAR_RELATIVE(l_diff_qr, l_diff_sc, prec);

    // test landmark state (as updated by back_substitute)
    const auto& lms_qr = bal_problem_qr.landmarks();
    const auto& lms_sc = bal_problem_sc.landmarks();
    for (size_t i = 0; i < lms_qr.size(); ++i) {
      const auto& lb_qr = lms_qr[i].p_w;
      const auto& lb_sc = lms_sc[i].p_w;

      ROOTBA_EXPECT_MAT_NEAR_RELATIVE(lb_qr, lb_sc, prec);
    }
  };

  // test w/o pose damping (landmark damping is set inside test_equivalent)
  test_equivalent();

  // test w/ pose damping
  lqr.set_pose_damping(lambda);
  lsc.set_pose_damping(lambda);
  test_equivalent();
}

#endif

}  // namespace rootba
