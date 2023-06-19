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
#include "rootba/sc/linearization_sc.hpp"
#include "rootba/testing/test_types.hpp"

template <typename Scalar_>
class BlockSparseMatrixTest : public ::testing::Test {
 public:
  using Scalar = Scalar_;
};

namespace rootba {

TYPED_TEST_SUITE(BlockSparseMatrixTest, ScalarTestTypes);

TYPED_TEST(BlockSparseMatrixTest, Multiply) {
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

  typename LinearizationSC<Scalar, POSE_SIZE>::Options lsc_options;
  LinearizationSC<Scalar, POSE_SIZE> lsc(bal_problem, lsc_options);
  lsc.linearize_problem();
  lsc.set_landmark_damping(damping);

  auto test_matrix_multiply = [&]() {
    using SparseMat = Eigen::SparseMatrix<Scalar, Eigen::RowMajor>;

    BlockSparseMatrix<Scalar> H_pp(lsc.num_cols_reduced(),
                                   lsc.num_cols_reduced());
    VecX b_p;
    lsc.get_Hb(H_pp, b_p);
    SparseMat H_pp_eigen = H_pp.get_sparse_matrix();

    // Check right_multiply
    VecX Hx = H_pp.right_multiply(b_p);
    VecX Hx_eigen = H_pp_eigen * b_p;
    EXPECT_TRUE(Hx.isApprox(Hx_eigen, prec))
        << "diff " << (Hx - Hx_eigen).norm() << " norm(Hx): " << Hx.norm()
        << " norm(Hx_eigen): " << Hx_eigen.norm();

    // Check left_multiply
    VecX Htx = H_pp.left_multiply(b_p);
    VecX Htx_eigen = H_pp_eigen.transpose() * b_p;
    EXPECT_TRUE(Htx.rows() == Htx_eigen.rows() &&
                Htx.cols() == Htx_eigen.cols())
        << "dim(Htx): [" << Htx.rows() << ", " << Htx.cols()
        << "] dim(Htx_eigen): " << Htx_eigen.rows() << ", " << Htx_eigen.cols()
        << "]";
    EXPECT_TRUE(Htx.isApprox(Htx_eigen, prec))
        << "diff " << (Htx - Htx_eigen).norm() << " norm(Htx): " << Htx.norm()
        << " norm(Htx_eigen): " << Htx_eigen.norm();
  };

  test_matrix_multiply();
  lsc.set_pose_damping(damping);
  test_matrix_multiply();
}

}  // namespace rootba
