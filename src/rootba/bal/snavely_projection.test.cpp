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

#include "rootba/bal/snavely_projection.hpp"

#include <basalt/camera/bal_camera.hpp>

#include "rootba/testing/test_types.hpp"

namespace rootba {

#if defined(ROOTBA_INSTANTIATIONS_FLOAT) || \
    defined(ROOTBA_INSTANTIATIONS_DOUBLE)

template <typename Scalar_>
class SnavelyProjectionTest : public ::testing::Test {
 public:
  using Scalar = Scalar_;
};

TYPED_TEST_SUITE(SnavelyProjectionTest, ScalarTestTypes);

TYPED_TEST(SnavelyProjectionTest, BasicUsage) {
  using Scalar = typename TestFixture::Scalar;
  using Vec3 = Vec<Scalar, 3>;
  using Vec2 = Vec<Scalar, 2>;

  basalt::BalCamera<Scalar> basalt_cam =
      basalt::BalCamera<Scalar>::getTestProjections().at(0);

  // test point projection
  Vec3 p = Vec3::Random();
  p.z() = std::abs(p.z()) + 0.1;
  Vec2 proj_basalt;
  EXPECT_TRUE(basalt_cam.project(p.homogeneous(), proj_basalt));

  // create a copy
  {
    SnavelyCameraModel<Scalar> cam;
    cam.params() = basalt_cam.getParam();

    // named accessors read
    EXPECT_EQ(cam.f(), cam.params()(0));
    EXPECT_EQ(cam.k1(), cam.params()(1));
    EXPECT_EQ(cam.k2(), cam.params()(2));

    // data access
    EXPECT_EQ(cam.params().data(), cam.data());
    EXPECT_TRUE(cam.params() == basalt_cam.getParam());

    // project test
    EXPECT_TRUE(proj_basalt.isApprox(cam.project(p)));

    // named accessors write
    cam.f() = 42;
    cam.k1() = 23;
    cam.k2() = 7;
    EXPECT_EQ(cam.f(), 42);
    EXPECT_EQ(cam.k1(), 23);
    EXPECT_EQ(cam.k2(), 7);

    // param write
    cam.params() = basalt_cam.getParam();
    EXPECT_TRUE(cam.params() == basalt_cam.getParam());
  }

  // create a non-const map
  {
    Vec3 params = basalt_cam.getParam();
    Eigen::Map<SnavelyCameraModel<Scalar>> cam(params.data());

    // named accessors read
    EXPECT_EQ(cam.f(), cam.params()(0));
    EXPECT_EQ(cam.k1(), cam.params()(1));
    EXPECT_EQ(cam.k2(), cam.params()(2));

    // data access
    EXPECT_EQ(cam.params().data(), cam.data());
    EXPECT_TRUE(cam.params() == basalt_cam.getParam());

    // project test
    EXPECT_TRUE(proj_basalt.isApprox(cam.project(p)));

    // named accessors write
    cam.f() = 42;
    cam.k1() = 23;
    cam.k2() = 7;
    EXPECT_EQ(cam.f(), 42);
    EXPECT_EQ(cam.k1(), 23);
    EXPECT_EQ(cam.k2(), 7);

    // writen paramters go through to param
    EXPECT_TRUE(cam.params() == params);

    // param write
    cam.params() = basalt_cam.getParam();
    EXPECT_TRUE(cam.params() == basalt_cam.getParam());
  }

  // create a const map
  {
    Eigen::Map<const SnavelyCameraModel<Scalar>> cam(
        basalt_cam.getParam().data());

    // parameter equality
    EXPECT_TRUE(cam.params() == basalt_cam.getParam());

    // named accessors read
    EXPECT_EQ(cam.f(), cam.params()(0));
    EXPECT_EQ(cam.k1(), cam.params()(1));
    EXPECT_EQ(cam.k2(), cam.params()(2));

    // data access
    EXPECT_EQ(cam.params().data(), cam.data());
    EXPECT_TRUE(cam.params() == basalt_cam.getParam());

    // project test
    EXPECT_TRUE(proj_basalt.isApprox(cam.project(p)));
  }
}

TYPED_TEST(SnavelyProjectionTest, Project) {
  using Scalar = typename TestFixture::Scalar;
  using Vec2 = Vec<Scalar, 2>;
  using Vec4 = Vec<Scalar, 4>;

  Eigen::aligned_vector<basalt::BalCamera<Scalar>> test_cams =
      basalt::BalCamera<Scalar>::getTestProjections();

  for (const auto& basalt_cam : test_cams) {
    SnavelyCameraModel<Scalar> cam;
    cam.params() = basalt_cam.getParam();

    for (int x = -10; x <= 10; x++) {
      for (int y = -10; y <= 10; y++) {
        for (int z = 0; z <= 5; z++) {
          Vec4 p(x, y, z, 0.23424);

          Vec4 p_normalized = Vec4::Zero();
          p_normalized.template head<3>() = p.template head<3>().normalized();
          Vec2 res;
          bool success = basalt_cam.project(p, res);

          if (success) {
            Vec2 res2 = cam.project(p.template head<3>().eval());

            EXPECT_TRUE(
                res.isApprox(res2, Sophus::Constants<Scalar>::epsilonSqrt()))
                << "res " << res.transpose() << " res2 " << res2.transpose();
          }
        }
      }
    }
  }
}

#endif

}  // namespace rootba
