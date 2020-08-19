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

#include <basalt/camera/bal_camera.hpp>
#include <ceres/autodiff_cost_function.h>

#include "rootba/ceres/types.hpp"
#include "rootba/util/eigen_types.hpp"

namespace rootba {

// ceres residual
template <int Options = 0>
class BalSnavelyReprojectionError {
 public:
  explicit BalSnavelyReprojectionError(const Vec2d& obs) : obs_(obs) {}

  template <class T>
  bool operator()(const T* camera, const T* landmark, T* residual) const {
    Eigen::Map<const Sophus::SE3<T>> T_c_w(camera);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> intrinsics(camera + 7);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_w(landmark);
    Eigen::Map<Eigen::Matrix<T, 2, 1>> res_vec(residual);

    const basalt::BalCamera<T> cam(intrinsics);

    auto p_cam = T_c_w * p_w;
    if constexpr (Options & VALID_PROJECTIONS_ONLY) {
      if (p_cam.z() < Sophus::Constants<T>::epsilonSqrt()) {
        // invalid projection
        res_vec.setZero();
      } else {
        Eigen::Matrix<T, 2, 1> p_proj;
        cam.project(p_cam.homogeneous(), p_proj);
        res_vec = p_proj - obs_;
      }
    } else {
      Eigen::Matrix<T, 2, 1> p_proj;
      cam.project(p_cam.homogeneous(), p_proj);
      res_vec = p_proj - obs_;
    }

    return true;
  }

  static ceres::CostFunction* create(const Vec2d& obs) {
    return (
        new ceres::AutoDiffCostFunction<BalSnavelyReprojectionError, 2, 10, 3>(
            new BalSnavelyReprojectionError(obs)));
  }

 private:
  Vec2d obs_;
};

}  // namespace rootba
