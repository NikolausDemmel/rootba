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
#include "rootba/bal/bal_bundle_adjustment_helper.hpp"

#include "rootba/testing/test_jacobian.hpp"
#include "rootba/testing/test_types.hpp"

namespace rootba {

#if defined(ROOTBA_INSTANTIATIONS_FLOAT) || \
    defined(ROOTBA_INSTANTIATIONS_DOUBLE)

template <typename Scalar_>
class BalBundleAdjustmentHelperTest : public ::testing::Test {
 public:
  using Scalar = Scalar_;
};

TYPED_TEST_SUITE(BalBundleAdjustmentHelperTest, ScalarTestTypes);

TYPED_TEST(BalBundleAdjustmentHelperTest, LinearizePoint) {
  using Scalar = typename TestFixture::Scalar;
  using SO3 = Sophus::SO3<Scalar>;
  using SE3 = Sophus::SE3<Scalar>;
  using Vec2 = Vec<Scalar, 2>;
  using Vec3 = Vec<Scalar, 3>;
  using Vec6 = Vec<Scalar, 6>;
  using Mat26 = Mat<Scalar, 2, 6>;
  using Mat23 = Mat<Scalar, 2, 3>;

  // relax precision a bit (to make float tests pass)
  const Scalar prec = Eigen::NumTraits<Scalar>::dummy_precision() * 2;

  Vec3 lm_p_w = Vec3::Random();
  lm_p_w.z() += 10;

  const basalt::BalCamera<Scalar> intr =
      basalt::BalCamera<Scalar>::getTestProjections()[0];
  const SE3 T_c_w(SO3::exp(Vec3::Random() / 100), Vec3::Random());

  Vec2 obs;
  intr.project((T_c_w * lm_p_w).homogeneous(), obs);
  Vec2 obs_noise = Vec2::Random();
  obs += obs_noise;

  Vec2 res;
  Mat26 d_res_d_xi;
  Mat23 d_res_d_i;
  Mat23 d_res_d_l;

  bool ignore_validity_check = false;

  bool valid = BalBundleAdjustmentHelper<Scalar>::linearize_point(
      obs, lm_p_w, T_c_w, intr, ignore_validity_check, res, &d_res_d_xi,
      &d_res_d_i, &d_res_d_l);
  EXPECT_TRUE(valid);

  EXPECT_TRUE(res.isApprox(-obs_noise, prec))
      << "res: " << res.transpose() << ", obs_noise: " << obs_noise
      << ", diff: " << (res + obs_noise).norm() << ", rel-diff: "
      << (res + obs_noise).norm() / std::max(res.norm(), obs_noise.norm());

  {
    Vec6 x0 = Vec6::Zero();
    test_jacobian(
        "d_res_d_xi", d_res_d_xi,
        [&](const Vec6& x) {
          SE3 T_c_w_new = T_c_w;
          BalProblem<Scalar>::Camera::inc_pose(x, T_c_w_new);

          Vec2 res;
          bool valid = BalBundleAdjustmentHelper<Scalar>::linearize_point(
              obs, lm_p_w, T_c_w_new, intr, ignore_validity_check, res);
          EXPECT_TRUE(valid);

          return res;
        },
        x0);
  }

  {
    Vec3 x0 = Vec3::Zero();
    test_jacobian(
        "d_res_d_i", d_res_d_i,
        [&](const Vec3& x) {
          basalt::BalCamera<Scalar> intr_new = intr;
          intr_new += x;

          Vec2 res;
          bool valid = BalBundleAdjustmentHelper<Scalar>::linearize_point(
              obs, lm_p_w, T_c_w, intr_new, ignore_validity_check, res);
          EXPECT_TRUE(valid);

          return res;
        },
        x0);
  }

  {
    Vec3 x0 = Vec3::Zero();
    test_jacobian(
        "d_res_d_l", d_res_d_l,
        [&](const Vec3& x) {
          Vec3 lm_p_w_new = lm_p_w + x;

          Vec2 res;
          bool valid = BalBundleAdjustmentHelper<Scalar>::linearize_point(
              obs, lm_p_w_new, T_c_w, intr, ignore_validity_check, res);
          EXPECT_TRUE(valid);

          return res;
        },
        x0);
  }
}

#endif

}  // namespace rootba
