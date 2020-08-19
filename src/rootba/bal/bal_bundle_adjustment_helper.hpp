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
#pragma once

#include "rootba/bal/bal_problem.hpp"
#include "rootba/bal/residual_info.hpp"
#include "rootba/bal/solver_options.hpp"

namespace rootba {

template <typename Scalar>
class BalBundleAdjustmentHelper {
 public:
  using IntrinsicsT = basalt::BalCamera<Scalar>;

  static constexpr size_t POSE_SIZE = 6;
  static constexpr size_t INTRINSICS_SIZE = 3;
  static constexpr size_t CAMERA_SIZE = POSE_SIZE + INTRINSICS_SIZE;
  static constexpr size_t LANDMARK_SIZE = 3;
  static constexpr size_t RESIDUAL_SIZE = 2;

  using Vec2 = Mat<Scalar, 2, 1>;
  using Vec3 = Mat<Scalar, 3, 1>;
  using Vec4 = Mat<Scalar, 4, 1>;
  using VecR = Mat<Scalar, RESIDUAL_SIZE, 1>;

  using Mat3 = Mat<Scalar, 3, 3>;
  using Mat4 = Mat<Scalar, 4, 4>;
  using Mat24 = Mat<Scalar, 2, 4>;
  using MatRP = Mat<Scalar, RESIDUAL_SIZE, POSE_SIZE>;
  using MatRI = Mat<Scalar, RESIDUAL_SIZE, INTRINSICS_SIZE>;
  using MatRL = Mat<Scalar, RESIDUAL_SIZE, LANDMARK_SIZE>;

  using SE3 = Sophus::SE3<Scalar>;
  using SO3 = Sophus::SO3<Scalar>;

  // compute error and residual weight (for jacobian) according to robust loss
  static std::tuple<Scalar, Scalar> compute_error_weight(
      const BalResidualOptions& options, Scalar res_squared);

  // compute the error for all observations
  static void compute_error(const BalProblem<Scalar>& bal_problem,
                            const SolverOptions& options, ResidualInfo& error);

  // linearize one observation
  static bool linearize_point(const Vec2& obs, const Vec3& lm_p_w,
                              const SE3& cam_T_c_w,
                              const basalt::BalCamera<Scalar>& intr,
                              bool ignore_validity_check, VecR& res,
                              MatRP* d_res_d_xi = nullptr,
                              MatRI* d_res_d_i = nullptr,
                              MatRL* d_res_d_l = nullptr);
};

}  // namespace rootba
