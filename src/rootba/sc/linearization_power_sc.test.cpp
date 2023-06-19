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
#include "rootba/sc/linearization_power_sc.hpp"

#include <type_traits>

#include "rootba/bal/bal_problem.hpp"
#include "rootba/cg/preconditioner.hpp"
#include "rootba/qr/linearization_utils.hpp"
#include "rootba/sc/linearization_sc.hpp"
#include "rootba/testing/eigen_utils.hpp"
#include "rootba/testing/float_utils.hpp"
#include "rootba/testing/test_types.hpp"

namespace rootba {

#if defined(ROOTBA_INSTANTIATIONS_FLOAT) || \
    defined(ROOTBA_INSTANTIATIONS_DOUBLE)

template <typename Scalar_>
class LinearizationPowerSCTest : public ::testing::Test {
 public:
  using Scalar = Scalar_;
};

constexpr int POSE_SIZE = 9;
const std::string TEST_DATASET_PATH =
    "data/rootba/test/bal-ladybug-problem-49-7776-pre-shrink-1800.txt";

TYPED_TEST_SUITE(LinearizationPowerSCTest, ScalarTestTypes);

// Test computation of Hpp_inv for LinearizationPowerSC by comparing it to the
// inverted JACOBI preconditioner of LinearizationSC.
TYPED_TEST(LinearizationPowerSCTest, PowerSCHppInvTest) {
  using Scalar = typename TestFixture::Scalar;
  using VecX = Vec<Scalar, Eigen::Dynamic>;

  // Expected accuracy of test depending on floating point precision.
  // Relax precision a bit from default, since comparison after inverse seems to
  // be a bit sensitive to exact addition associativity (default: 1e-5F, 1e-12).
  const auto prec = precision_float_or_double<Scalar>(1e-2F, 2e-11);

  // not too small damping to see the effect of errors in implementation
  const Scalar lambda = 1e3;

  // test with default jacobian scaling epsilon
  const auto jacobi_scaling_eps = default_testing_jacobi_scaling_eps<Scalar>();

  // load dataset
  auto bal_problem =
      load_normalized_bal_problem_quiet<Scalar>(TEST_DATASET_PATH);
  const size_t num_cams = bal_problem.num_cameras();

  // lsc options
  typename LinearizationSC<Scalar, POSE_SIZE>::Options lsc_options;
  lsc_options.jacobi_scaling_eps = jacobi_scaling_eps;

  // explicit
  LinearizationSC<Scalar, POSE_SIZE> lsc(bal_problem, lsc_options);
  lsc.linearize_problem();
  lsc.set_landmark_damping(lambda);
  lsc.scale_Jl_cols();
  lsc.compute_Jp_scale_and_scale_Jp_cols();

  // lpsc options
  typename LinearizationPowerSC<Scalar, POSE_SIZE>::Options lpsc_options;
  lpsc_options.jacobi_scaling_eps = jacobi_scaling_eps;

  // power
  LinearizationPowerSC<Scalar, POSE_SIZE> lpsc(bal_problem, lpsc_options);
  lpsc.linearize_problem();
  lpsc.set_landmark_damping(lambda);
  lpsc.scale_Jl_cols();
  lpsc.compute_Jp_scale_and_scale_Jp_cols();

  auto do_test = [&]() {
    // compute and invert JACOBI preconditioner
    const auto jacobi = lsc.get_jacobi();
    const auto pred = BlockDiagonalPreconditioner<Scalar>(
        num_cams, POSE_SIZE, jacobi.block_diagonal, nullptr);

    // Compute and cache Hpp_inv
    VecX b_p_power;
    lpsc.prepare_Hb(b_p_power);

    // test Hpp_inv block-wise against JACOBI preconditioner
    for (size_t i = 0; i < num_cams; ++i) {
      const auto Hpp_inv = lpsc.get_Hpp_inv(i);
      const auto jacobi_inv =
          pred.inv_blocks.template block<POSE_SIZE, POSE_SIZE>(POSE_SIZE * i,
                                                               0);

      ROOTBA_EXPECT_MAT_NEAR_RELATIVE(Hpp_inv, jacobi_inv, prec);
    }
  };

  // test w/o pose damping (landmark damping is set above)
  do_test();

  // test w/ pose damping
  lsc.set_pose_damping(lambda);
  lpsc.set_pose_damping(lambda);
  do_test();
}

// Test components of LinearizationPowerSC by comparing right_multiply (which is
// not used in actual PowerSC computation, but just there for testing)
// to LinearizationSC.
TYPED_TEST(LinearizationPowerSCTest, PowerSCRightMultipleTest) {
  using Scalar = typename TestFixture::Scalar;
  using VecX = Vec<Scalar, Eigen::Dynamic>;

  // expected accuracy of test depending on floating point precision
  const auto prec = default_test_precision<Scalar>();

  // not too small damping to see the effect of errors in implementation
  const Scalar lambda = 1e-1;

  // test with default jacobian scaling epsilon
  const auto jacobi_scaling_eps = default_testing_jacobi_scaling_eps<Scalar>();

  // load dataset
  auto bal_problem =
      load_normalized_bal_problem_quiet<Scalar>(TEST_DATASET_PATH);
  const size_t num_cams = bal_problem.num_cameras();

  // lsc options
  typename LinearizationSC<Scalar, POSE_SIZE>::Options lsc_options;
  lsc_options.jacobi_scaling_eps = jacobi_scaling_eps;

  // explicit
  LinearizationSC<Scalar, POSE_SIZE> lsc(bal_problem, lsc_options);
  lsc.linearize_problem();
  lsc.set_landmark_damping(lambda);
  lsc.scale_Jl_cols();
  lsc.compute_Jp_scale_and_scale_Jp_cols();

  // lpsc options
  typename LinearizationPowerSC<Scalar, POSE_SIZE>::Options lpsc_options;
  lpsc_options.jacobi_scaling_eps = jacobi_scaling_eps;

  // power
  LinearizationPowerSC<Scalar, POSE_SIZE> lpsc(bal_problem, lpsc_options);
  lpsc.linearize_problem();
  lpsc.set_landmark_damping(lambda);
  lpsc.scale_Jl_cols();
  lpsc.compute_Jp_scale_and_scale_Jp_cols();

  auto do_test = [&]() {
    // get H and b of RCS
    BlockSparseMatrix<Scalar> H_pp_explicit(lsc.num_cols_reduced(),
                                            lsc.num_cols_reduced());
    VecX b_p_explicit;
    lsc.get_Hb(H_pp_explicit, b_p_explicit);

    VecX b_p_power;
    lpsc.prepare_Hb(b_p_power);

    ROOTBA_EXPECT_MAT_NEAR_RELATIVE(b_p_explicit, b_p_power, prec);

    // right_multiply with random vector
    const VecX x = VecX::Random(POSE_SIZE * num_cams);
    const VecX res_explicit = H_pp_explicit.right_multiply(x);
    const VecX res_power = lpsc.right_multiply(x);

    // test right_multiply
    ROOTBA_EXPECT_MAT_NEAR_RELATIVE(res_explicit, res_power, prec);
  };

  // test w/o pose damping (landmark damping is set above)
  do_test();

  // test w/ pose damping
  lsc.set_pose_damping(lambda);
  lpsc.set_pose_damping(lambda);
  do_test();
}

// Test solve for LinearizationPowerSC by manual computation of the
// corresponding steps for m=0 and m=5.
TYPED_TEST(LinearizationPowerSCTest, PowerSCSolveTest) {
  using Scalar = typename TestFixture::Scalar;
  using VecX = Vec<Scalar, Eigen::Dynamic>;

  // expected accuracy of test depending on floating point precision
  // Relax precision a bit from default (default: 1e-5F, 1e-12).
  const auto prec = precision_float_or_double<Scalar>(1e-4F, 1e-12);

  // not too small damping to see the effect of errors in implementation
  const Scalar lambda = 1e-1;

  // test with default jacobian scaling epsilon
  const auto jacobi_scaling_eps = default_testing_jacobi_scaling_eps<Scalar>();

  // load dataset
  auto bal_problem =
      load_normalized_bal_problem_quiet<Scalar>(TEST_DATASET_PATH);

  // lpsc options
  typename LinearizationPowerSC<Scalar, POSE_SIZE>::Options lpsc_options;
  lpsc_options.jacobi_scaling_eps = jacobi_scaling_eps;
  std::unique_ptr<LinearizationPowerSC<Scalar, POSE_SIZE>> lpsc;

  auto do_test = [&]() {
    // i = 0
    {
      lpsc_options.power_order = 0;
      lpsc.reset(new LinearizationPowerSC<Scalar, POSE_SIZE>(bal_problem,
                                                             lpsc_options));
      lpsc->linearize_problem();
      lpsc->set_landmark_damping(lambda);
      lpsc->scale_Jl_cols();
      lpsc->compute_Jp_scale_and_scale_Jp_cols();

      VecX b_p;
      lpsc->prepare_Hb(b_p);

      // x = Hpp^-1 * -x
      const VecX res_expected = lpsc->right_mul_Hpp_inv(-b_p);

      typename LinearizationPowerSC<Scalar, POSE_SIZE>::PerSolveOptions pso = {
          -1, -1, nullptr};
      VecX res_actual;
      lpsc->solve(b_p, res_actual, pso);
      ROOTBA_EXPECT_MAT_NEAR_RELATIVE(res_expected, res_actual, prec);
    }

    // i = 5
    {
      lpsc_options.power_order = 5;
      lpsc.reset(new LinearizationPowerSC<Scalar, POSE_SIZE>(bal_problem,
                                                             lpsc_options));
      lpsc->linearize_problem();
      lpsc->set_landmark_damping(lambda);
      lpsc->scale_Jl_cols();
      lpsc->compute_Jp_scale_and_scale_Jp_cols();

      VecX b_p;
      lpsc->prepare_Hb(b_p);

      // x = Hpp^-1 * -x
      const VecX res_0 = lpsc->right_mul_Hpp_inv(-b_p);
      // x = (Hpp^-1 * Hpl * Hll^-1 * Hlp)^i * x
      const VecX res_1 = lpsc->right_mul_Hpp_inv(lpsc->right_mul_e0(res_0));
      const VecX res_2 = lpsc->right_mul_Hpp_inv(lpsc->right_mul_e0(res_1));
      const VecX res_3 = lpsc->right_mul_Hpp_inv(lpsc->right_mul_e0(res_2));
      const VecX res_4 = lpsc->right_mul_Hpp_inv(lpsc->right_mul_e0(res_3));
      const VecX res_5 = lpsc->right_mul_Hpp_inv(lpsc->right_mul_e0(res_4));
      const VecX res_expected =
          ((((res_0 + res_1) + res_2) + res_3) + res_4) + res_5;

      typename LinearizationPowerSC<Scalar, POSE_SIZE>::PerSolveOptions pso = {
          -1, -1, nullptr};
      VecX res_actual;
      lpsc->solve(b_p, res_actual, pso);
      ROOTBA_EXPECT_MAT_NEAR_RELATIVE(res_expected, res_actual, prec);
    }
  };

  // test w/o pose damping (landmark damping is set above)
  do_test();

  // test w/ pose damping
  lpsc->set_pose_damping(lambda);
  do_test();
}

#endif

}  // namespace rootba
