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
#pragma once

#include <fstream>
#include <mutex>

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

    // 0: parallel_reduce (may use more memory)
    // 1: parallel_for with mutex
    int reduction_alg = 1;

    size_t power_order = 20;
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

  inline size_t num_poses() const { return pose_idx_.size(); }

  inline const std::vector<size_t>& get_pose_idx() const { return pose_idx_; }

  inline auto get_Jli(const size_t obs_idx) const {
    return storage_.template block<2, 3>(2 * obs_idx, lm_idx_);
  }

  inline auto get_Jl() const {
    return storage_.template middleCols<3>(lm_idx_);
  }

  inline auto get_Jpi(const size_t obs_idx) const {
    return storage_.template block<2, POSE_SIZE>(2 * obs_idx, 0);
  }

  inline auto get_Hll_inv() const { return Hll_inv_; }

  // Fill the explicit reduced H, b linear system by parallel_reduce
  inline void add_Hb(BlockSparseMatrix<Scalar>& accu, VecX& b) const {
    ROOTBA_ASSERT(state_ == LINEARIZED);

    // Compute landmark-landmark block plus inverse
    Vec3 Hll_inv_bl;
    {
      auto Jl = storage_.block(0, lm_idx_, num_rows_, 3);
      Hll_inv_ = Jl.transpose() * Jl;
      Hll_inv_.diagonal().array() += lambda_;
      Hll_inv_ = Hll_inv_.inverse().eval();

      Hll_inv_bl = Hll_inv_ * (Jl.transpose() * storage_.col(res_idx_));
    }

    // Add pose-pose blocks and
    for (size_t i = 0; i < pose_idx_.size(); i++) {
      size_t cam_idx_i = pose_idx_[i];

      auto Jp_i = storage_.template block<2, POSE_SIZE>(2 * i, 0);
      auto Jl_i = storage_.template block<2, 3>(2 * i, lm_idx_);
      auto r_i = storage_.template block<2, 1>(2 * i, res_idx_);

      MatX Hpp = Jp_i.transpose() * Jp_i;
      accu.add(cam_idx_i, cam_idx_i, std::move(Hpp));

      // Schur complement blocks
      for (size_t j = 0; j < pose_idx_.size(); j++) {
        size_t cam_idx_j = pose_idx_[j];
        auto Jp_j = storage_.template block<2, POSE_SIZE>(2 * j, 0);
        auto Jl_j = storage_.template block<2, 3>(2 * j, lm_idx_);

        MatX Hpl_Hll_inv_Hlp =
            -Jp_i.transpose() * (Jl_i * (Hll_inv_ * (Jl_j.transpose() * Jp_j)));

        accu.add(cam_idx_i, cam_idx_j, std::move(Hpl_Hll_inv_Hlp));
      }

      // add blocks to b
      b.template segment<POSE_SIZE>(cam_idx_i * POSE_SIZE) +=
          Jp_i.transpose() * (r_i - Jl_i * Hll_inv_bl);
    }
  }

  // Fill the explicit reduced H, b linear system by parallel_for and mutex
  inline void add_Hb(BlockSparseMatrix<Scalar>& accu, VecX& b,
                     std::vector<std::mutex>& Hpp_mutex,
                     std::vector<std::mutex>& pose_mutex,
                     const size_t num_cam) const {
    ROOTBA_ASSERT(state_ == LINEARIZED);

    // Compute landmark-landmark block plus inverse
    Vec3 Hll_inv_bl;
    {
      auto Jl = storage_.block(0, lm_idx_, num_rows_, 3);
      Hll_inv_ = Jl.transpose() * Jl;
      Hll_inv_.diagonal().array() += lambda_;
      Hll_inv_ = Hll_inv_.inverse().eval();

      Hll_inv_bl = Hll_inv_ * (Jl.transpose() * storage_.col(res_idx_));
    }

    // Add pose-pose blocks and
    for (size_t i = 0; i < pose_idx_.size(); i++) {
      const size_t cam_idx_i = pose_idx_[i];

      auto Jp_i = storage_.template block<2, POSE_SIZE>(2 * i, 0);
      auto Jl_i = storage_.template block<2, 3>(2 * i, lm_idx_);
      auto r_i = storage_.template block<2, 1>(2 * i, res_idx_);

      // TODO: check (Niko: not sure what we wanted to check here...)

      {
        MatX Hpp = Jp_i.transpose() * Jp_i;
        std::scoped_lock lock(Hpp_mutex.at(cam_idx_i * num_cam + cam_idx_i));
        accu.add(cam_idx_i, cam_idx_i, std::move(Hpp));
      }

      // Schur complement blocks
      for (size_t j = 0; j < pose_idx_.size(); j++) {
        const size_t cam_idx_j = pose_idx_[j];
        auto Jp_j = storage_.template block<2, POSE_SIZE>(2 * j, 0);
        auto Jl_j = storage_.template block<2, 3>(2 * j, lm_idx_);

        {
          MatX Hpl_Hll_inv_Hlp =
              -Jp_i.transpose() *
              (Jl_i * (Hll_inv_ * (Jl_j.transpose() * Jp_j)));

          std::scoped_lock lock(Hpp_mutex.at(cam_idx_i * num_cam + cam_idx_j));
          accu.add(cam_idx_i, cam_idx_j, std::move(Hpl_Hll_inv_Hlp));
        }
      }

      // add blocks to b
      {
        std::scoped_lock lock(pose_mutex.at(cam_idx_i));
        b.template segment<POSE_SIZE>(cam_idx_i * POSE_SIZE) +=
            Jp_i.transpose() * (r_i - Jl_i * Hll_inv_bl);
      }
    }
  }

  // Compute b and cache Hll_inv for solving RCS, and optionally compute Hpp and
  // the diagonal blocks of H.
  inline void stage(VecX& b, RowMatX* Hpp,
                    BlockDiagonalAccumulator<Scalar>* H_diag,
                    std::vector<std::mutex>* pose_mutex = nullptr) const {
    // fill Hll_inv: (Jl'Jl + lm_lambda * I)^-1
    const auto Jl = storage_.block(0, lm_idx_, num_rows_, 3);
    Hll_inv_ = Jl.transpose() * Jl;
    Hll_inv_.diagonal().array() += lambda_;
    Hll_inv_ = Hll_inv_.inverse().eval();
    const Vec3 Hll_inv_bl =
        Hll_inv_ * (Jl.transpose() * storage_.col(res_idx_));

    // Compute different componments depending on the given parameters
    if (H_diag) {
      // b, Hll_inv_bl, Hpp, and H_diag (optional)
      add_Hpp_H_diag_b(b, Hll_inv_bl, Hpp, *H_diag, pose_mutex);
    } else if (Hpp) {
      // b, Hll_inv_bl, and Hpp (optional)
      add_Hpp_b(b, Hll_inv_bl, *Hpp, pose_mutex);
    } else {
      // b, Hll_inv_bl,
      add_b(b, Hll_inv_bl, pose_mutex);
    }
  }

  inline void add_Hpp(BlockDiagonalAccumulator<Scalar>& Hpp,
                      std::vector<std::mutex>& pose_mutex) const {
    ROOTBA_ASSERT(state_ == LINEARIZED);

    for (size_t i = 0; i < pose_idx_.size(); ++i) {
      const size_t cam_idx = pose_idx_[i];
      const auto Jp = storage_.template block<2, POSE_SIZE>(2 * i, 0);
      const MatX tmp = Jp.transpose() * Jp;
      {
        std::scoped_lock lock(pose_mutex[cam_idx]);
        Hpp.add(cam_idx, tmp);
      }
    }
  }

  inline void add_Jp_x(VecX& res, const VecX& x) const {
    ROOTBA_ASSERT(state_ == LINEARIZED);

    for (size_t i = 0; i < pose_idx_.size(); ++i) {
      const auto u = get_Jpi(i);
      const auto v = x.template segment<POSE_SIZE>(pose_idx_[i] * POSE_SIZE);
      res.template segment<2>(i * 2) = u * v;
    }
  }

  inline void add_JpT_x(VecX& res, const VecX& x,
                        std::vector<std::mutex>* pose_mutex = nullptr) const {
    ROOTBA_ASSERT(state_ == LINEARIZED);

    for (size_t i = 0; i < pose_idx_.size(); ++i) {
      const auto u = get_Jpi(i);
      const auto v = x.template segment<2>(i * 2);
      const size_t idx = pose_idx_[i];

      if (pose_mutex) {
        std::scoped_lock lock((*pose_mutex)[idx]);
        res.template segment<POSE_SIZE>(idx * POSE_SIZE) += u.transpose() * v;
      } else {
        res.template segment<POSE_SIZE>(idx * POSE_SIZE) += u.transpose() * v;
      }
    }
  }

  void back_substitute(const VecX& pose_inc, Scalar& l_diff) {
    ROOTBA_ASSERT(state_ == LINEARIZED);

    Mat3 Hll = Mat3::Zero();
    Vec3 tmp = Vec3::Zero();
    VecX J_inc;
    J_inc.setZero(num_rows_);

    // Add pose-pose blocks and
    for (size_t i = 0; i < pose_idx_.size(); i++) {
      size_t cam_idx_i = pose_idx_[i];

      auto Jp_i = storage_.template block<2, POSE_SIZE>(2 * i, 0);
      auto Jl_i = storage_.template block<2, 3>(2 * i, lm_idx_);
      auto r_i = storage_.template block<2, 1>(2 * i, res_idx_);

      Hll += Jl_i.transpose() * Jl_i;

      auto p_inc = pose_inc.template segment<POSE_SIZE>(cam_idx_i * POSE_SIZE);

      tmp += Jl_i.transpose() * (r_i + Jp_i * p_inc);
      J_inc.template segment<2>(2 * i) += Jp_i * p_inc;
    }

    // TODO: store additionally "Hllinv" (inverted with lambda), so we don't
    // need lambda in the interface
    Hll.diagonal().array() += lambda_;
    Vec3 inc = -Hll.inverse() * tmp;

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
  inline void add_b(VecX& b, const Vec3& Hll_inv_bl,
                    std::vector<std::mutex>* pose_mutex) const {
    for (size_t i = 0; i < pose_idx_.size(); ++i) {
      const size_t cam_idx = pose_idx_[i];
      const auto Jp = storage_.template block<2, POSE_SIZE>(2 * i, 0);
      const auto Jl = storage_.template block<2, 3>(2 * i, lm_idx_);
      const auto r = storage_.template block<2, 1>(2 * i, res_idx_);

      const VecX b_tmp = Jp.transpose() * (r - Jl * Hll_inv_bl);
      if (pose_mutex) {
        std::scoped_lock lock((*pose_mutex)[cam_idx]);
        b.template segment<POSE_SIZE>(cam_idx * POSE_SIZE) += b_tmp;
      } else {
        b.template segment<POSE_SIZE>(cam_idx * POSE_SIZE) += b_tmp;
      }
    }
  }

  inline void add_Hpp_b(VecX& b, const Vec3& Hll_inv_bl, RowMatX& Hpp,
                        std::vector<std::mutex>* pose_mutex) const {
    for (size_t i = 0; i < pose_idx_.size(); ++i) {
      const size_t cam_idx = pose_idx_[i];
      const auto Jp = storage_.template block<2, POSE_SIZE>(2 * i, 0);
      const auto Jl = storage_.template block<2, 3>(2 * i, lm_idx_);
      const auto r = storage_.template block<2, 1>(2 * i, res_idx_);

      const VecX b_tmp = Jp.transpose() * (r - Jl * Hll_inv_bl);
      const RowMatX Hpp_tmp = Jp.transpose() * Jp;
      if (pose_mutex) {
        std::scoped_lock lock((*pose_mutex)[cam_idx]);
        b.template segment<POSE_SIZE>(cam_idx * POSE_SIZE) += b_tmp;
        Hpp.template block<POSE_SIZE, POSE_SIZE>(cam_idx * POSE_SIZE, 0) +=
            Hpp_tmp;
      } else {
        b.template segment<POSE_SIZE>(cam_idx * POSE_SIZE) += b_tmp;
        Hpp.template block<POSE_SIZE, POSE_SIZE>(cam_idx * POSE_SIZE, 0) +=
            Hpp_tmp;
      }
    }
  }

  inline void add_Hpp_H_diag_b(VecX& b, const Vec3& Hll_inv_bl, RowMatX* Hpp,
                               BlockDiagonalAccumulator<Scalar>& H_diag,
                               std::vector<std::mutex>* pose_mutex) const {
    for (size_t i = 0; i < pose_idx_.size(); ++i) {
      const size_t cam_idx = pose_idx_[i];
      const auto Jp = storage_.template block<2, POSE_SIZE>(2 * i, 0);
      const auto Jl = storage_.template block<2, 3>(2 * i, lm_idx_);
      const auto r = storage_.template block<2, 1>(2 * i, res_idx_);

      const VecX b_tmp = Jp.transpose() * (r - Jl * Hll_inv_bl);
      const auto Jp_t_Jl = Jp.transpose() * Jl;

      const RowMatX Hpp_tmp = Jp.transpose() * Jp;
      MatX H_diag_tmp = Hpp_tmp - Jp_t_Jl * (Hll_inv_ * Jp_t_Jl.transpose());
      if (pose_mutex) {
        std::scoped_lock lock((*pose_mutex)[cam_idx]);

        b.template segment<POSE_SIZE>(cam_idx * POSE_SIZE) += b_tmp;
        H_diag.add(cam_idx, std::move(H_diag_tmp));
        if (Hpp) {
          Hpp->template block<POSE_SIZE, POSE_SIZE>(cam_idx * POSE_SIZE, 0) +=
              Hpp_tmp;
        }
      } else {
        b.template segment<POSE_SIZE>(cam_idx * POSE_SIZE) += b_tmp;
        H_diag.add(cam_idx, std::move(H_diag_tmp));
        if (Hpp) {
          Hpp->template block<POSE_SIZE, POSE_SIZE>(cam_idx * POSE_SIZE, 0) +=
              Hpp_tmp;
        }
      }
    }
  }

  // Dense storage for pose Jacobians, padding, landmark Jacobians and
  // residuals [Jp (all jacobians in one column) | Jl | res]
  RowMatX storage_;
  // The value is computed and cache while preparing the RCS (H, b)
  mutable Mat3 Hll_inv_;

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
