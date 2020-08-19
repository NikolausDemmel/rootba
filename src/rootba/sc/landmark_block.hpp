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

#include <fstream>

#include <Eigen/Dense>

#include "rootba/bal/bal_bundle_adjustment_helper.hpp"
#include "rootba/bal/bal_problem.hpp"
#include "rootba/bal/bal_residual_options.hpp"
#include "rootba/cg/block_sparse_matrix.hpp"
#include "rootba/util/assert.hpp"
#include "rootba/util/format.hpp"

namespace rootba {

template <typename Scalar, int POSE_SIZE>
class LandmarkBlockSC {
 public:
  struct Options {
    // use Householder instead of Givens for marginalization
    bool use_householder = true;

    // use_valid_projections_only: if true, set invalid projection's
    // residual and jacobian to 0; invalid means z <= 0
    bool use_valid_projections_only = true;

    // huber norm with given threshold, else squared norm
    BalResidualOptions residual_options;

    // ceres uses 1.0 / (1.0 + sqrt(SquaredColumnNorm))
    // we use 1.0 / (eps + sqrt(SquaredColumnNorm))
    Scalar jacobi_scaling_eps = 1.0;
  };

  enum State { UNINITIALIZED = 0, ALLOCATED, NUMERICAL_FAILURE, LINEARIZED };

  inline bool is_numerical_failure() const {
    return state_ == NUMERICAL_FAILURE;
  }

  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
  using Vec4 = Eigen::Matrix<Scalar, 4, 1>;

  using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
  using Mat36 = Eigen::Matrix<Scalar, 3, 6>;

  using MatX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
  using RowMatX =
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  using Landmark = typename BalProblem<Scalar>::Landmark;
  using Camera = typename BalProblem<Scalar>::Camera;
  using Landmarks = typename BalProblem<Scalar>::Landmarks;
  using Cameras = typename BalProblem<Scalar>::Cameras;

  inline void allocate_landmark(Landmark& lm, const Options& options) {
    options_ = options;

    pose_idx_.clear();
    pose_idx_.reserve(lm.obs.size());
    for (const auto& [cam_idx, obs] : lm.obs) {
      pose_idx_.push_back(cam_idx);
    }

    num_rows_ = pose_idx_.size() * 2;

    lm_idx_ = POSE_SIZE;
    res_idx_ = lm_idx_ + 3;
    num_cols_ = res_idx_ + 1;

    storage_.resize(num_rows_, num_cols_);

    state_ = ALLOCATED;

    lm_ptr_ = &lm;
  }

  // may set state to NumericalFailure --> linearization at this state is
  // unusable. Numeric check is only performed for residuals that were
  // considered to be used (valid), which depends on use_valid_projections_only
  // setting.
  inline void linearize_landmark(const Cameras& cameras) {
    ROOTBA_ASSERT(state_ == ALLOCATED || state_ == NUMERICAL_FAILURE ||
                  state_ == LINEARIZED);

    storage_.setZero(num_rows_, num_cols_);

    bool numerically_valid = true;

    for (size_t i = 0; i < pose_idx_.size(); i++) {
      size_t cam_idx = pose_idx_[i];
      size_t obs_idx = i * 2;
      size_t pose_idx = 0;

      const auto& obs = lm_ptr_->obs.at(cam_idx);
      const auto& cam = cameras.at(cam_idx);

      typename BalBundleAdjustmentHelper<Scalar>::MatRP Jp;
      typename BalBundleAdjustmentHelper<Scalar>::MatRI Ji;
      typename BalBundleAdjustmentHelper<Scalar>::MatRL Jl;

      Vec2 res;
      const bool valid = BalBundleAdjustmentHelper<Scalar>::linearize_point(
          obs.pos, lm_ptr_->p_w, cam.T_c_w, cam.intrinsics, true, res, &Jp, &Ji,
          &Jl);

      if (!options_.use_valid_projections_only || valid) {
        numerically_valid = numerically_valid && Jl.array().isFinite().all() &&
                            Jp.array().isFinite().all() &&
                            Ji.array().isFinite().all() &&
                            res.array().isFinite().all();

        const Scalar res_squared = res.squaredNorm();
        const auto [weighted_error, weight] =
            BalBundleAdjustmentHelper<Scalar>::compute_error_weight(
                options_.residual_options, res_squared);
        const Scalar sqrt_weight = std::sqrt(weight);

        storage_.template block<2, 6>(obs_idx, pose_idx) = sqrt_weight * Jp;
        storage_.template block<2, 3>(obs_idx, pose_idx + 6) = sqrt_weight * Ji;
        storage_.template block<2, 3>(obs_idx, lm_idx_) = sqrt_weight * Jl;
        storage_.template block<2, 1>(obs_idx, res_idx_) = sqrt_weight * res;
      }
    }

    if (numerically_valid) {
      state_ = LINEARIZED;
    } else {
      state_ = NUMERICAL_FAILURE;
    }
  }

  inline void add_Jp_diag2(VecX& res) const {
    ROOTBA_ASSERT(state_ == LINEARIZED);

    for (size_t i = 0; i < pose_idx_.size(); i++) {
      size_t cam_idx = pose_idx_[i];
      res.template segment<POSE_SIZE>(POSE_SIZE * cam_idx) +=
          storage_.template block<2, POSE_SIZE>(2 * i, 0)
              .colwise()
              .squaredNorm();
    }
  }

  inline void scale_Jl_cols() {
    ROOTBA_ASSERT(state_ == LINEARIZED);

    // ceres uses 1.0 / (1.0 + sqrt(SquaredColumnNorm))
    // we use 1.0 / (eps + sqrt(SquaredColumnNorm))
    Jl_col_scale =
        (options_.jacobi_scaling_eps +
         storage_.block(0, lm_idx_, num_rows_, 3).colwise().norm().array())
            .inverse();

    storage_.block(0, lm_idx_, num_rows_, 3) *= Jl_col_scale.asDiagonal();
  }

  inline void scale_Jp_cols(const VecX& jacobian_scaling) {
    ROOTBA_ASSERT(state_ == LINEARIZED);

    for (size_t i = 0; i < pose_idx_.size(); i++) {
      size_t cam_idx = pose_idx_[i];

      storage_.template block<2, POSE_SIZE>(2 * i, 0) *=
          jacobian_scaling.template segment<POSE_SIZE>(POSE_SIZE * cam_idx)
              .asDiagonal();
    }
  }

  inline State get_state() const { return state_; }

  inline void set_landmark_damping(Scalar lambda) { lambda_ = lambda; }

  // Fill the reduced H, b linear system.
  inline void add_Hb(BlockSparseMatrix<Scalar>& accu, VecX& b) const {
    ROOTBA_ASSERT(state_ == LINEARIZED);

    // Compute landmark-landmark block plus inverse
    Mat3 H_ll;
    Mat3 H_ll_inv;
    Vec3 H_ll_inv_bl;
    {
      auto Jl = storage_.block(0, lm_idx_, num_rows_, 3);
      H_ll = Jl.transpose() * Jl;
      H_ll.diagonal().array() += lambda_;
      H_ll_inv = H_ll.inverse();

      H_ll_inv_bl = H_ll_inv * Jl.transpose() * storage_.col(res_idx_);
    }

    // Add pose-pose blocks and
    for (size_t i = 0; i < pose_idx_.size(); i++) {
      size_t cam_idx_i = pose_idx_[i];

      auto Jp_i = storage_.template block<2, POSE_SIZE>(2 * i, 0);
      auto Jl_i = storage_.template block<2, 3>(2 * i, lm_idx_);
      auto r_i = storage_.template block<2, 1>(2 * i, res_idx_);

      MatX H_pp = Jp_i.transpose() * Jp_i;
      accu.add(cam_idx_i, cam_idx_i, std::move(H_pp));

      // Schur complement blocks
      for (size_t j = 0; j < pose_idx_.size(); j++) {
        size_t cam_idx_j = pose_idx_[j];
        auto Jp_j = storage_.template block<2, POSE_SIZE>(2 * j, 0);
        auto Jl_j = storage_.template block<2, 3>(2 * j, lm_idx_);

        MatX H_pl_H_ll_inv_H_lp =
            -Jp_i.transpose() * Jl_i * H_ll_inv * Jl_j.transpose() * Jp_j;

        accu.add(cam_idx_i, cam_idx_j, std::move(H_pl_H_ll_inv_H_lp));
      }

      // add blocks to b
      b.template segment<POSE_SIZE>(cam_idx_i * POSE_SIZE) +=
          Jp_i.transpose() * (r_i - Jl_i * H_ll_inv_bl);
    }
  }

  void back_substitute(const VecX& pose_inc, Scalar& l_diff) {
    ROOTBA_ASSERT(state_ == LINEARIZED);

    Mat3 H_ll = Mat3::Zero();
    Vec3 tmp = Vec3::Zero();
    VecX J_inc;
    J_inc.setZero(num_rows_);

    // Add pose-pose blocks and
    for (size_t i = 0; i < pose_idx_.size(); i++) {
      size_t cam_idx_i = pose_idx_[i];

      auto Jp_i = storage_.template block<2, POSE_SIZE>(2 * i, 0);
      auto Jl_i = storage_.template block<2, 3>(2 * i, lm_idx_);
      auto r_i = storage_.template block<2, 1>(2 * i, res_idx_);

      H_ll += Jl_i.transpose() * Jl_i;

      auto p_inc = pose_inc.template segment<POSE_SIZE>(cam_idx_i * POSE_SIZE);

      tmp += Jl_i.transpose() * (r_i + Jp_i * p_inc);
      J_inc.template segment<2>(2 * i) += Jp_i * p_inc;
    }

    // TODO: store additionally "Hllinv" (inverted with lambda), so we don't
    // need lambda in the interface
    H_ll.diagonal().array() += lambda_;
    Vec3 inc = -H_ll.inverse() * tmp;

    // Add landmark jacobian cost change
    J_inc += storage_.block(0, lm_idx_, num_rows_, 3) * inc;

    l_diff -= J_inc.transpose() * (0.5 * J_inc + storage_.col(res_idx_));

    // Note: scale only after computing model cost change
    inc.array() *= Jl_col_scale.array();
    lm_ptr_->p_w += inc;
  }

  void print_storage(const std::string& filename) const {
    std::ofstream f(filename);

    Eigen::IOFormat clean_fmt(4, 0, " ", "\n", "", "");

    f << "Storage (state: " << state_
      << " Jl_col_scale: " << Jl_col_scale.transpose() << "):\n"
      << storage_.format(clean_fmt) << std::endl;

    f.close();
  }

 protected:
  // Dense storage for pose Jacobians, padding, landmark Jacobians and
  // residuals [J_p (all jacobians in one column) | J_l | res]
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      storage_;

  Vec3 Jl_col_scale;
  Scalar lambda_ = 0;

  std::vector<size_t> pose_idx_;
  size_t lm_idx_ = 0;
  size_t res_idx_ = 0;

  size_t num_cols_ = 0;
  size_t num_rows_ = 0;

  Options options_;

  State state_ = UNINITIALIZED;

  Landmark* lm_ptr_ = nullptr;
};

}  // namespace rootba
