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

#include "rootba/qr/linearization_qr.hpp"

#include "rootba/bal/bal_problem.hpp"
#include "rootba/util/test_utils.hpp"

namespace rootba {

#if defined(ROOTBA_INSTANTIATIONS_FLOAT) || \
    defined(ROOTBA_INSTANTIATIONS_DOUBLE)

template <typename Scalar_>
class LinearizationQRTest : public ::testing::Test {
 public:
  using Scalar = Scalar_;
};

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

#endif

}  // namespace rootba
