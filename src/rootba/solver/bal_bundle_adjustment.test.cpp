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

#include "rootba/bal/bal_problem.hpp"
#include "rootba/qr/linearization_qr.hpp"
#include "rootba/sc/linearization_sc.hpp"
#include "rootba/testing/test_types.hpp"

namespace rootba {

#if defined(ROOTBA_INSTANTIATIONS_FLOAT) || \
    defined(ROOTBA_INSTANTIATIONS_DOUBLE)

template <typename Scalar_>
class BalBundleAdjustmentTest : public ::testing::Test {
 public:
  using Scalar = Scalar_;
};

TYPED_TEST_SUITE(BalBundleAdjustmentTest, ScalarTestTypes);

TYPED_TEST(BalBundleAdjustmentTest, QrScEquivalenceTest) {
  using Scalar = typename TestFixture::Scalar;
  using VecX = Vec<Scalar, Eigen::Dynamic>;

  // relax precision a bit (to make float tests pass)
  const Scalar prec = Eigen::NumTraits<Scalar>::dummy_precision() * 2;

  const std::string test_dataset_path =
      "data/rootba/test/bal-ladybug-problem-49-7776-pre-shrink-1800.txt";

  const Scalar damping = 1e-4;
  constexpr int POSE_SIZE = 9;

  // load dataset
  auto bal_problem =
      load_normalized_bal_problem_quiet<Scalar>(test_dataset_path);

  typename LinearizationQR<Scalar, POSE_SIZE>::Options lqr_options;
  LinearizationQR<Scalar, POSE_SIZE> lqr(bal_problem, lqr_options);
  lqr.linearize_problem();

  typename LinearizationSC<Scalar, POSE_SIZE>::Options lsc_options;
  LinearizationSC<Scalar, POSE_SIZE> lsc(bal_problem, lsc_options);
  lsc.linearize_problem();

  // Check diagonal norms and scale pose variables
  VecX pose_jacobian_scaling;
  {
    VecX diag2_qr = lqr.get_Jp_diag2();
    VecX diag2_sc = lsc.get_Jp_diag2();
    EXPECT_TRUE(diag2_qr.isApprox(diag2_sc, prec))
        << "diff " << (diag2_qr - diag2_sc).norm();

    pose_jacobian_scaling = (1e-6 + diag2_qr.array().sqrt()).inverse();
  }

  lqr.scale_Jl_cols();
  lqr.perform_qr();
  lqr.scale_Jp_cols(pose_jacobian_scaling);
  lqr.set_landmark_damping(damping);

  lsc.scale_Jl_cols();
  lsc.scale_Jp_cols(pose_jacobian_scaling);
  lsc.set_landmark_damping(damping);

  auto test_linearization = [&]() {
    using SparseMat = Eigen::SparseMatrix<Scalar, Eigen::RowMajor>;

    BlockSparseMatrix<Scalar> H_pp(lqr.num_cols(), lqr.num_cols());
    VecX b_p_sc;
    lsc.get_Hb(H_pp, b_p_sc);

    // Check H_pp
    auto Q2T_Jp = lqr.get_Q2TJp();
    SparseMat H_pp_qr = Q2T_Jp.transpose() * Q2T_Jp;
    SparseMat H_pp_sc = H_pp.get_sparse_matrix();
    SparseMat H_pp_diff = (H_pp_sc - H_pp_qr);
    EXPECT_TRUE(H_pp_sc.isApprox(H_pp_qr, prec))
        << "diff " << H_pp_diff.norm() << " norm(H_pp_sc): " << H_pp_sc.norm()
        << " norm(H_pp_qr): " << H_pp_qr.norm();

    // Check b_p
    VecX b_p_qr = lqr.get_Q2TJp_T_Q2Tr();
    EXPECT_TRUE(b_p_sc.isApprox(b_p_qr, prec))
        << "diff " << (b_p_sc - b_p_qr).norm()
        << " norm(b_p_sc): " << b_p_sc.norm()
        << " norm(b_p_qr): " << b_p_qr.norm();

    // Check block diagonal for pre-conditioner
    IndexedBlocks<Scalar> block_diag = lqr.get_Q2TJp_T_Q2TJp_blockdiag();
    for (const auto& [idx, block] : block_diag) {
      const auto& block2 = H_pp.block_storage.at(idx);
      EXPECT_TRUE(block.isApprox(block2, prec))
          << "diff " << (block - block2).norm()
          << " norm(block): " << block.norm()
          << " norm(block2): " << block2.norm();
    }
  };

  test_linearization();

  lqr.set_pose_damping(damping);
  lsc.set_pose_damping(damping);

  test_linearization();
}

#endif

}  // namespace rootba
