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

#include "rootba/bal/bal_bundle_adjustment_helper.hpp"
#include "rootba/qr/landmark_block_base.hpp"
#include "rootba/util/assert.hpp"
#include "rootba/util/format.hpp"

namespace rootba {

namespace {  // helper

template <class Derived, class Scalar>
void add_triplets_dense(const Eigen::DenseBase<Derived>& mat,
                        const size_t row_offset, const size_t col_offset,
                        std::vector<Eigen::Triplet<Scalar>>& triplets) {
  for (Eigen::Index row = 0; row < mat.rows(); ++row) {
    for (Eigen::Index col = 0; col < mat.cols(); ++col) {
      triplets.emplace_back(row + row_offset, col + col_offset, mat(row, col));
    }
  }
}

}  // namespace

template <class T, typename Scalar, int POSE_SIZE>
bool LandmarkBlockBase<T, Scalar, POSE_SIZE>::is_numerical_failure() const {
  return state_ == State::NUMERICAL_FAILURE;
}

template <class T, typename Scalar, int POSE_SIZE>
void LandmarkBlockBase<T, Scalar, POSE_SIZE>::allocate_landmark(
    LandmarkBlockBase::Landmark& lm,
    const LandmarkBlockBase::Options& options) {
  options_ = options;

  // verify that num_obs >= 2, such that num_rows >= 4+3
  ROOTBA_ASSERT_MSG(lm.obs.size() >= 2,
                    "Current implemention of LandmarkBlock for QR solver "
                    "requires at least 2 observations per landmark.");

  static_cast<T*>(this)->allocate_landmark_impl(lm);

  damping_rotations_.clear();
  damping_rotations_.reserve(6);

  state_ = State::ALLOCATED;

  lm_ptr_ = &lm;
}

template <class T, typename Scalar, int POSE_SIZE>
void LandmarkBlockBase<T, Scalar, POSE_SIZE>::linearize_landmark(
    const LandmarkBlockBase::Cameras& cameras) {
  ROOTBA_ASSERT(state_ == State::ALLOCATED ||
                state_ == State::NUMERICAL_FAILURE ||
                state_ == State::LINEARIZED || state_ == State::MARGINALIZED);

  const size_t lm_idx = static_cast<const T*>(this)->get_lm_idx();
  const size_t res_idx = static_cast<const T*>(this)->get_res_idx();
  const auto& pose_idx = static_cast<const T*>(this)->get_pose_idx();
  auto& storage = static_cast<T*>(this)->get_storage();
  storage.setZero();

  damping_rotations_.clear();
  damping_rotations_.reserve(6);

  bool numerically_valid = true;

  for (size_t i = 0; i < pose_idx.size(); i++) {
    size_t cam_idx = pose_idx[i];
    size_t obs_idx = i * 2;
    size_t pose_idx = i * POSE_SIZE;

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

      storage.template block<2, 6>(obs_idx, pose_idx) = sqrt_weight * Jp;
      storage.template block<2, 3>(obs_idx, pose_idx + 6) = sqrt_weight * Ji;
      storage.template block<2, 3>(obs_idx, lm_idx) = sqrt_weight * Jl;
      storage.template block<2, 1>(obs_idx, res_idx) = sqrt_weight * res;
    }
  }

  if (numerically_valid) {
    state_ = State::LINEARIZED;
  } else {
    state_ = State::NUMERICAL_FAILURE;
  }
}

template <class T, typename Scalar, int POSE_SIZE>
void LandmarkBlockBase<T, Scalar, POSE_SIZE>::perform_qr() {
  ROOTBA_ASSERT(state_ == State::LINEARIZED);

  // Since we use dense matrices Householder QR might be better:
  // https://mathoverflow.net/questions/227543/why-householder-reflection-is-better-than-givens-rotation-in-dense-linear-algebr

  if (options_.use_householder) {
    perform_qr_householder();
  } else {
    perform_qr_givens();
  }

  state_ = State::MARGINALIZED;
}

template <class T, typename Scalar, int POSE_SIZE>
void LandmarkBlockBase<T, Scalar, POSE_SIZE>::set_landmark_damping(
    Scalar lambda) {
  ROOTBA_ASSERT(state_ == State::MARGINALIZED);
  ROOTBA_ASSERT(lambda >= 0);

  const size_t lm_idx = static_cast<T*>(this)->get_lm_idx();
  const size_t num_rows = static_cast<T*>(this)->get_num_rows();
  auto& storage = static_cast<T*>(this)->get_storage();

  if (has_landmark_damping()) {
    ROOTBA_ASSERT(damping_rotations_.size() == 6);

    // undo dampening
    for (int n = 2; n >= 0; n--) {
      for (int m = n; m >= 0; m--) {
        storage.applyOnTheLeft(num_rows - 3 + n - m, n,
                               damping_rotations_.back().adjoint());
        damping_rotations_.pop_back();
      }
    }
  }

  if (lambda == 0) {
    storage.template block<3, 3>(num_rows - 3, lm_idx).diagonal().setZero();
  } else {
    ROOTBA_ASSERT(Jl_col_scale_.array().isFinite().all());

    storage.template block<3, 3>(num_rows - 3, lm_idx)
        .diagonal()
        .setConstant(sqrt(lambda));

    ROOTBA_ASSERT(damping_rotations_.empty());

    // apply dampening and remember rotations to undo
    for (int n = 0; n < 3; n++) {
      for (int m = 0; m <= n; m++) {
        damping_rotations_.emplace_back();
        damping_rotations_.back().makeGivens(
            storage(n, lm_idx + n), storage(num_rows - 3 + n - m, lm_idx + n));
        storage.applyOnTheLeft(num_rows - 3 + n - m, n,
                               damping_rotations_.back());
      }
    }
  }
}

template <class T, typename Scalar, int POSE_SIZE>
void LandmarkBlockBase<T, Scalar, POSE_SIZE>::back_substitute(
    const LandmarkBlockBase::VecX& pose_inc, Scalar& l_diff) {
  ROOTBA_ASSERT(state_ == State::MARGINALIZED);

  const size_t lm_idx = static_cast<T*>(this)->get_lm_idx();
  const size_t res_idx = static_cast<T*>(this)->get_res_idx();
  const size_t padding_idx = static_cast<T*>(this)->get_padding_idx();
  const size_t padding_size = static_cast<T*>(this)->get_padding_size();
  const size_t num_rows = static_cast<T*>(this)->get_num_rows();
  const auto& pose_idx = static_cast<T*>(this)->get_pose_idx();
  const auto& storage = static_cast<T*>(this)->get_storage();

  VecX pose_inc_reduced(padding_idx + padding_size);
  for (size_t i = 0; i < pose_idx.size(); i++) {
    size_t cam_idx = pose_idx[i];
    pose_inc_reduced.template segment<POSE_SIZE>(POSE_SIZE * i) =
        pose_inc.template segment<POSE_SIZE>(POSE_SIZE * cam_idx);
  }
  pose_inc_reduced.tail(padding_size).setConstant(0);

  const auto Q1T_Jl = storage.template block<3, 3>(0, lm_idx)
                          .template triangularView<Eigen::Upper>();

  const auto Q1T_Jp = storage.topLeftCorner(3, padding_idx + padding_size);
  const auto Q1T_r = storage.col(res_idx).template head<3>();

  Vec3 inc = -Q1T_Jl.solve(Q1T_r + Q1T_Jp * pose_inc_reduced);

  // We want to compute the model cost change. The model function is
  //
  //     L(inc) = F(x) + incT JT r + 0.5 incT JT J inc
  //
  // and thus the expect decrease in cost for the computed increment is
  //
  //     l_diff = L(0) - L(inc)
  //            = - incT JT r - 0.5 incT JT J inc.
  //            = - incT JT (r + 0.5 J inc)
  //            = - (J inc)T (r + 0.5 (J inc))

  // undo damping before we compute the model cost difference
  set_landmark_damping(0);

  VecX QT_J_inc =
      storage.topLeftCorner(num_rows - 3, padding_idx + padding_size) *
      pose_inc_reduced;

  QT_J_inc.template head<3>() += Q1T_Jl * inc;

  auto QT_r = storage.col(res_idx).head(num_rows - 3);
  l_diff -= QT_J_inc.transpose() * (0.5 * QT_J_inc + QT_r);

  // TODO: detect and handle case like ceres, allowing a few iterations but
  // stopping eventually
  if (!inc.array().isFinite().all() || !lm_ptr_->p_w.array().isFinite().all()) {
    std::cout << "=================================" << std::endl;
    std::cout << "inc\n" << inc.transpose() << std::endl;
    std::cout << "lm_ptr->p_w\n" << lm_ptr_->p_w.transpose() << std::endl;
    // std::cout << "QT_Jl\n" << QT_Jl << std::endl;
    std::cout << "get_Q1Tr\n" << get_Q1Tr().transpose() << std::endl;
    std::cout << "get_Q1TJp_postmult_x(pose_inc)\n"
              << get_Q1TJp_postmult_x(pose_inc).transpose() << std::endl;

    std::cout << "Storage_lm\n" << storage.rightCols(4) << std::endl;

    std::cout.flush();
    LOG(FATAL) << "Numerical failure";
  }

  // Note: scale only after computing model cost change
  inc.array() *= Jl_col_scale_.array();
  lm_ptr_->p_w += inc;
}

template <class T, typename Scalar, int POSE_SIZE>
size_t LandmarkBlockBase<T, Scalar, POSE_SIZE>::num_reduced_cams() const {
  const auto& pose_idx = static_cast<const T*>(this)->get_pose_idx();
  return pose_idx.size();
}

template <class T, typename Scalar, int POSE_SIZE>
size_t LandmarkBlockBase<T, Scalar, POSE_SIZE>::num_Q2T_rows() const {
  const size_t num_rows = static_cast<const T*>(this)->get_num_rows();
  return num_rows - 3;
}

template <class T, typename Scalar, int POSE_SIZE>
typename LandmarkBlockBase<T, Scalar, POSE_SIZE>::Vec3
LandmarkBlockBase<T, Scalar, POSE_SIZE>::get_Q1Tr() const {
  const size_t res_idx = static_cast<const T*>(this)->get_res_idx();
  const auto& storage = static_cast<const T*>(this)->get_storage();

  return storage.col(res_idx).template head<3>();
}

template <class T, typename Scalar, int POSE_SIZE>
void LandmarkBlockBase<T, Scalar, POSE_SIZE>::add_triplets_Q2TJp(
    size_t row_offset, std::vector<Eigen::Triplet<Scalar>>& triplets) const {
  ROOTBA_ASSERT(state_ == State::MARGINALIZED);

  const auto& pose_idx = static_cast<const T*>(this)->get_pose_idx();
  const auto& storage = static_cast<const T*>(this)->get_storage();

  // TODO: check if it makes a difference if we order the triples by rows
  for (size_t i = 0; i < pose_idx.size(); ++i) {
    size_t col_offset = pose_idx[i] * POSE_SIZE;
    add_triplets_dense(
        storage.block(3, i * POSE_SIZE, num_Q2T_rows(), POSE_SIZE), row_offset,
        col_offset, triplets);
  }
}

template <class T, typename Scalar, int POSE_SIZE>
typename LandmarkBlockBase<T, Scalar, POSE_SIZE>::Vec3
LandmarkBlockBase<T, Scalar, POSE_SIZE>::get_Q1TJp_postmult_x(
    const LandmarkBlockBase::VecX& x_pose) const {
  ROOTBA_ASSERT(state_ == State::MARGINALIZED);

  const size_t padding_idx = static_cast<const T*>(this)->get_padding_idx();
  const size_t padding_size = static_cast<const T*>(this)->get_padding_size();
  const auto& pose_idx = static_cast<const T*>(this)->get_pose_idx();
  const auto& storage = static_cast<const T*>(this)->get_storage();

  VecX x_pose_reduced(padding_idx + padding_size);

  for (size_t i = 0; i < pose_idx.size(); i++) {
    size_t cam_idx = pose_idx[i];
    x_pose_reduced.template segment<POSE_SIZE>(POSE_SIZE * i) =
        x_pose.template segment<POSE_SIZE>(POSE_SIZE * cam_idx);
  }
  x_pose_reduced.tail(padding_size).setConstant(0);

  Vec3 res =
      storage.topLeftCorner(3, padding_idx + padding_size) * x_pose_reduced;

  return res;
}

template <class T, typename Scalar, int POSE_SIZE>
typename LandmarkBlockBase<T, Scalar, POSE_SIZE>::VecX
LandmarkBlockBase<T, Scalar, POSE_SIZE>::get_Q2TJp_postmult_x(
    const LandmarkBlockBase::VecX& x_pose) const {
  ROOTBA_ASSERT(state_ == State::MARGINALIZED);

  const size_t padding_idx = static_cast<const T*>(this)->get_padding_idx();
  const size_t padding_size = static_cast<const T*>(this)->get_padding_size();
  const size_t num_rows = static_cast<const T*>(this)->get_num_rows();
  const auto& pose_idx = static_cast<const T*>(this)->get_pose_idx();
  const auto& storage = static_cast<const T*>(this)->get_storage();

  VecX x_pose_reduced(padding_idx + padding_size);

  for (size_t i = 0; i < pose_idx.size(); i++) {
    size_t cam_idx = pose_idx[i];
    x_pose_reduced.template segment<POSE_SIZE>(POSE_SIZE * i) =
        x_pose.template segment<POSE_SIZE>(POSE_SIZE * cam_idx);
  }
  x_pose_reduced.tail(padding_size).setConstant(0);

  VecX res =
      storage.bottomLeftCorner(num_rows - 3, padding_idx + padding_size) *
      x_pose_reduced;

  return res;
}

template <class T, typename Scalar, int POSE_SIZE>
void LandmarkBlockBase<T, Scalar, POSE_SIZE>::add_Q2TJp_premult_x(
    VecX& res, const VecX& x_r) const {
  ROOTBA_ASSERT(state_ == State::MARGINALIZED);

  const size_t padding_idx = static_cast<const T*>(this)->get_padding_idx();
  const size_t padding_size = static_cast<const T*>(this)->get_padding_size();
  const size_t num_rows = static_cast<const T*>(this)->get_num_rows();
  const auto& pose_idx = static_cast<const T*>(this)->get_pose_idx();
  const auto& storage = static_cast<const T*>(this)->get_storage();

  VecX res_reduced =
      x_r.transpose() *
      storage.bottomLeftCorner(num_rows - 3, padding_idx + padding_size);

  for (size_t i = 0; i < pose_idx.size(); i++) {
    size_t cam_idx = pose_idx[i];
    res.template segment<POSE_SIZE>(POSE_SIZE * cam_idx) +=
        res_reduced.template segment<POSE_SIZE>(POSE_SIZE * i);
  }
}

template <class T, typename Scalar, int POSE_SIZE>
void LandmarkBlockBase<T, Scalar, POSE_SIZE>::add_Q2TJp_T_Q2TJp_mult_x(
    LandmarkBlockBase::VecX& res, const LandmarkBlockBase::VecX& x_pose,
    std::vector<std::mutex>* pose_mutex) const {
  ROOTBA_ASSERT(state_ == State::MARGINALIZED);
  ROOTBA_ASSERT(res.size() == x_pose.size());

  const size_t padding_idx = static_cast<const T*>(this)->get_padding_idx();
  const size_t padding_size = static_cast<const T*>(this)->get_padding_size();
  const size_t num_rows = static_cast<const T*>(this)->get_num_rows();
  const auto& pose_idx = static_cast<const T*>(this)->get_pose_idx();
  const auto& storage = static_cast<const T*>(this)->get_storage();

  VecX x_pose_reduced(padding_idx + padding_size);

  for (size_t i = 0; i < pose_idx.size(); i++) {
    size_t cam_idx = pose_idx[i];
    x_pose_reduced.template segment<POSE_SIZE>(POSE_SIZE * i) =
        x_pose.template segment<POSE_SIZE>(POSE_SIZE * cam_idx);
  }
  x_pose_reduced.tail(padding_size).setConstant(0);

  const auto& block =
      storage.bottomLeftCorner(num_rows - 3, padding_idx + padding_size);

  VecX tmp = block * x_pose_reduced;
  x_pose_reduced.noalias() = block.adjoint() * tmp;

  for (size_t i = 0; i < pose_idx.size(); i++) {
    size_t cam_idx = pose_idx[i];

    if (pose_mutex == nullptr) {
      res.template segment<POSE_SIZE>(POSE_SIZE * cam_idx) +=
          x_pose_reduced.template segment<POSE_SIZE>(POSE_SIZE * i);
    } else {
      std::scoped_lock lock(pose_mutex->at(cam_idx));

      res.template segment<POSE_SIZE>(POSE_SIZE * cam_idx) +=
          x_pose_reduced.template segment<POSE_SIZE>(POSE_SIZE * i);
    }
  }
}

template <class T, typename Scalar, int POSE_SIZE>
void LandmarkBlockBase<T, Scalar, POSE_SIZE>::add_Q2TJp_T_Q2Tr(
    LandmarkBlockBase::VecX& res) const {
  ROOTBA_ASSERT(state_ == State::MARGINALIZED);

  const size_t padding_idx = static_cast<const T*>(this)->get_padding_idx();
  const size_t padding_size = static_cast<const T*>(this)->get_padding_size();
  const size_t num_rows = static_cast<const T*>(this)->get_num_rows();
  const size_t res_idx = static_cast<const T*>(this)->get_res_idx();
  const auto& pose_idx = static_cast<const T*>(this)->get_pose_idx();
  const auto& storage = static_cast<const T*>(this)->get_storage();

  VecX x_pose_reduced =
      storage.bottomLeftCorner(num_rows - 3, padding_idx + padding_size)
          .adjoint() *
      storage.col(res_idx).tail(num_rows - 3);
  // (Q2^T * Jp)^T * Q2^Tr

  for (size_t i = 0; i < pose_idx.size(); i++) {
    size_t cam_idx = pose_idx[i];
    res.template segment<POSE_SIZE>(POSE_SIZE * cam_idx) +=
        x_pose_reduced.template segment<POSE_SIZE>(POSE_SIZE * i);
  }
}

template <class T, typename Scalar, int POSE_SIZE>
void LandmarkBlockBase<T, Scalar, POSE_SIZE>::add_Q2TJp_diag2(
    LandmarkBlockBase::VecX& res) const {
  ROOTBA_ASSERT(state_ == State::MARGINALIZED);

  const size_t padding_idx = static_cast<const T*>(this)->get_padding_idx();
  const size_t padding_size = static_cast<const T*>(this)->get_padding_size();
  const size_t num_rows = static_cast<const T*>(this)->get_num_rows();
  const auto& pose_idx = static_cast<const T*>(this)->get_pose_idx();
  const auto& storage = static_cast<const T*>(this)->get_storage();

  // Note: even if no lm damping is set, this is fine, since last three rows
  // will just contain zeros
  VecX res_reduced =
      storage.bottomLeftCorner(num_rows - 3, padding_idx + padding_size)
          .colwise()
          .squaredNorm();

  for (size_t i = 0; i < pose_idx.size(); i++) {
    size_t cam_idx = pose_idx[i];
    res.template segment<POSE_SIZE>(POSE_SIZE * cam_idx) +=
        res_reduced.template segment<POSE_SIZE>(POSE_SIZE * i);
  }
}

template <class T, typename Scalar, int POSE_SIZE>
void LandmarkBlockBase<T, Scalar, POSE_SIZE>::add_Jp_diag2(
    LandmarkBlockBase::VecX& res) const {
  ROOTBA_ASSERT(state_ == State::LINEARIZED);

  const size_t padding_idx = static_cast<const T*>(this)->get_padding_idx();
  const size_t padding_size = static_cast<const T*>(this)->get_padding_size();
  const size_t num_rows = static_cast<const T*>(this)->get_num_rows();
  const auto& pose_idx = static_cast<const T*>(this)->get_pose_idx();
  const auto& storage = static_cast<const T*>(this)->get_storage();

  // TODO: we know that every column has only 2 non-zero rows, so maybe we
  // should limit computation to those...

  // exclude landmark damping rows (but they'd be 0 anyway)
  VecX res_reduced =
      storage.topLeftCorner(num_rows - 3, padding_idx + padding_size)
          .colwise()
          .squaredNorm();

  for (size_t i = 0; i < pose_idx.size(); i++) {
    size_t cam_idx = pose_idx[i];
    res.template segment<POSE_SIZE>(POSE_SIZE * cam_idx) +=
        res_reduced.template segment<POSE_SIZE>(POSE_SIZE * i);
  }
}

template <class T, typename Scalar, int POSE_SIZE>
void LandmarkBlockBase<T, Scalar, POSE_SIZE>::add_Q2TJp_T_Q2TJp_blockdiag(
    BlockDiagonalAccumulator<Scalar>& accu,
    std::vector<std::mutex>* pose_mutex) const {
  ROOTBA_ASSERT(state_ == State::MARGINALIZED);

  const size_t num_rows = static_cast<const T*>(this)->get_num_rows();
  const auto& pose_idx = static_cast<const T*>(this)->get_pose_idx();
  const auto& storage = static_cast<const T*>(this)->get_storage();

  // we include the dampening rows (may be zeros if no dampening set)
  if (pose_mutex) {
    for (size_t i = 0; i < pose_idx.size(); i++) {
      // using auto gives us a "reference" to the block
      const auto Q2T_Jp =
          storage.block(3, POSE_SIZE * i, num_rows - 3, POSE_SIZE);

      const size_t cam_idx = pose_idx[i];
      {
        MatX tmp = Q2T_Jp.transpose() * Q2T_Jp;
        std::scoped_lock lock(pose_mutex->at(cam_idx));
        accu.add(cam_idx, std::move(tmp));
      }
    }
  } else {
    for (size_t i = 0; i < pose_idx.size(); i++) {
      // using auto gives us a "reference" to the block
      auto Q2T_Jp = storage.block(3, POSE_SIZE * i, num_rows - 3, POSE_SIZE);
      const size_t cam_idx = pose_idx[i];
      accu.add(cam_idx, Q2T_Jp.transpose() * Q2T_Jp);
    }
  }
}

template <class T, typename Scalar, int POSE_SIZE>
void LandmarkBlockBase<T, Scalar, POSE_SIZE>::add_Jp_T_Jp_blockdiag(
    BlockDiagonalAccumulator<Scalar>& accu) const {
  ROOTBA_ASSERT(state_ == State::LINEARIZED);

  const auto& pose_idx = static_cast<const T*>(this)->get_pose_idx();
  const auto& storage = static_cast<const T*>(this)->get_storage();

  for (size_t i = 0; i < pose_idx.size(); i++) {
    // using auto gives us a "reference" to the block
    auto Jp = storage.block(2 * i, POSE_SIZE * i, 2, POSE_SIZE);

    size_t cam_idx = pose_idx[i];
    accu.add(cam_idx, Jp.transpose() * Jp);
  }
}

template <class T, typename Scalar, int POSE_SIZE>
void LandmarkBlockBase<T, Scalar, POSE_SIZE>::scale_Jl_cols() {
  ROOTBA_ASSERT(state_ == State::LINEARIZED);

  const size_t lm_idx = static_cast<T*>(this)->get_lm_idx();
  const size_t num_rows = static_cast<T*>(this)->get_num_rows();
  auto& storage = static_cast<T*>(this)->get_storage();

  // ceres uses 1.0 / (1.0 + sqrt(SquaredColumnNorm))
  // we use 1.0 / (eps + sqrt(SquaredColumnNorm))
  Jl_col_scale_ =
      (options_.jacobi_scaling_eps +
       storage.block(0, lm_idx, num_rows - 3, 3).colwise().norm().array())
          .inverse();

  storage.block(0, lm_idx, num_rows - 3, 3) *= Jl_col_scale_.asDiagonal();
}

template <class T, typename Scalar, int POSE_SIZE>
void LandmarkBlockBase<T, Scalar, POSE_SIZE>::scale_Jp_cols(
    const LandmarkBlockBase::VecX& jacobian_scaling) {
  ROOTBA_ASSERT(state_ == State::MARGINALIZED);

  const size_t padding_idx = static_cast<const T*>(this)->get_padding_idx();
  const size_t padding_size = static_cast<const T*>(this)->get_padding_size();
  const size_t num_rows = static_cast<const T*>(this)->get_num_rows();
  const auto& pose_idx = static_cast<const T*>(this)->get_pose_idx();
  auto& storage = static_cast<T*>(this)->get_storage();

  // we assume we apply scaling before damping (we exclude the last 3 rows)
  ROOTBA_ASSERT(!has_landmark_damping());

  VecX jacobian_scaling_reduced(padding_idx + padding_size);

  for (size_t i = 0; i < pose_idx.size(); i++) {
    size_t cam_idx = pose_idx[i];
    jacobian_scaling_reduced.template segment<POSE_SIZE>(POSE_SIZE * i) =
        jacobian_scaling.template segment<POSE_SIZE>(POSE_SIZE * cam_idx);
  }
  jacobian_scaling_reduced.tail(padding_size).setConstant(0);

  storage.topLeftCorner(num_rows - 3, padding_idx + padding_size) *=
      jacobian_scaling_reduced.asDiagonal();
}

template <class T, typename Scalar, int POSE_SIZE>
bool LandmarkBlockBase<T, Scalar, POSE_SIZE>::has_landmark_damping() const {
  return !damping_rotations_.empty();
}

template <class T, typename Scalar, int POSE_SIZE>
void LandmarkBlockBase<T, Scalar, POSE_SIZE>::print_storage(
    const std::string& filename) const {
  const auto& storage = static_cast<const T*>(this)->get_storage();

  std::ofstream f(filename);

  Eigen::IOFormat clean_fmt(4, 0, " ", "\n", "", "");

  f << "Storage (state: " << state_
    << ", damping: " << (has_landmark_damping() ? "yes" : "no")
    << " Jl_col_scale: " << Jl_col_scale_.transpose() << "):\n"
    << storage.format(clean_fmt) << std::endl;

  f.close();
}

template <class T, typename Scalar, int POSE_SIZE>
void LandmarkBlockBase<T, Scalar, POSE_SIZE>::stage2(
    Scalar lambda, const LandmarkBlockBase::VecX* jacobian_scaling,
    BlockDiagonalAccumulator<Scalar>* precond_block_diagonal,
    LandmarkBlockBase::VecX& bref) {
  // 1. scale jacobian
  if (jacobian_scaling) {
    scale_Jp_cols(*jacobian_scaling);
  }

  // 2. dampen landmarks
  set_landmark_damping(lambda);

  // 3b. compute block diagonal preconditioner (SCHUR_JACOBI)
  if (precond_block_diagonal) {
    add_Q2TJp_T_Q2TJp_blockdiag(*precond_block_diagonal);
  }

  // 4. compute rhs of reduced camera normal equations
  add_Q2TJp_T_Q2Tr(bref);
}

template <class T, typename Scalar, int POSE_SIZE>
typename LandmarkBlockBase<T, Scalar, POSE_SIZE>::RowMatX
LandmarkBlockBase<T, Scalar, POSE_SIZE>::get_Q1TJp(size_t num_cams) const {
  ROOTBA_ASSERT(state_ == State::Marginalized);

  const auto& pose_idx = static_cast<const T*>(this)->get_pose_idx();
  const auto& storage = static_cast<const T*>(this)->get_storage();

  RowMatX Q1T_Jp(3, POSE_SIZE * num_cams);
  Q1T_Jp.setZero();
  for (size_t i = 0; i < pose_idx.size(); ++i) {
    Q1T_Jp.block(0, pose_idx[i] * POSE_SIZE, 3, POSE_SIZE) =
        storage.block(0, i * POSE_SIZE, 3, POSE_SIZE);
  }
  return Q1T_Jp;
}

template <class T, typename Scalar, int POSE_SIZE>
typename LandmarkBlockBase<T, Scalar, POSE_SIZE>::RowMatX
LandmarkBlockBase<T, Scalar, POSE_SIZE>::get_Q2TJp(size_t num_cams) const {
  ROOTBA_ASSERT(state_ == State::Marginalized);

  const auto& pose_idx = static_cast<const T*>(this)->get_pose_idx();
  const auto& storage = static_cast<const T*>(this)->get_storage();

  RowMatX Q2T_Jp(num_Q2T_rows(), POSE_SIZE * num_cams);
  Q2T_Jp.setZero();
  for (size_t i = 0; i < pose_idx.size(); ++i) {
    MatX tmp = storage.block(3, i * POSE_SIZE, num_Q2T_rows(), POSE_SIZE);
    Q2T_Jp.block(0, pose_idx[i] * POSE_SIZE, num_Q2T_rows(), POSE_SIZE) = tmp;
  }
  return Q2T_Jp;
}

template <class T, typename Scalar, int POSE_SIZE>
typename LandmarkBlockBase<T, Scalar, POSE_SIZE>::State
LandmarkBlockBase<T, Scalar, POSE_SIZE>::get_state() const {
  return state_;
}

template <class T, typename Scalar, int POSE_SIZE>
void LandmarkBlockBase<T, Scalar, POSE_SIZE>::perform_qr_givens() {
  const size_t lm_idx = static_cast<T*>(this)->get_lm_idx();
  const size_t num_rows = static_cast<T*>(this)->get_num_rows();
  auto& storage = static_cast<T*>(this)->get_storage();

  // Based on "Matrix Computations 4th Edition by Golub and Van Loan"
  // See page 252, Algorithm 5.2.4 for how these two loops work
  Eigen::JacobiRotation<Scalar> gr;
  for (size_t n = 0; n < 3; n++) {
    for (size_t m = num_rows - 4; m > n; m--) {
      gr.makeGivens(storage(m - 1, lm_idx + n), storage(m, lm_idx + n));
      storage.applyOnTheLeft(m, m - 1, gr);
    }
  }
}

template <class T, typename Scalar, int POSE_SIZE>
void LandmarkBlockBase<T, Scalar, POSE_SIZE>::perform_qr_householder() {
  const size_t num_cols = static_cast<T*>(this)->get_num_cols();
  const size_t num_rows = static_cast<T*>(this)->get_num_rows();
  const size_t lm_idx = static_cast<T*>(this)->get_lm_idx();
  auto& storage = static_cast<T*>(this)->get_storage();

  VecX temp_vector1(num_cols);
  VecX temp_vector2(num_rows - 3);

  // Assumption: We have at least 2 observations, such that we have at least 4
  // rows + 3 for damping and remaining_rows is positive.
  ROOTBA_ASSERT(num_rows >= 4 + 3);

  for (size_t k = 0; k < 3; ++k) {
    size_t remaining_rows = num_rows - k - 3;

    Scalar beta;
    Scalar tau;
    storage.col(lm_idx + k)
        .segment(k, remaining_rows)
        .makeHouseholder(temp_vector2, tau, beta);

    storage.block(k, 0, remaining_rows, num_cols)
        .applyHouseholderOnTheLeft(temp_vector2, tau, temp_vector1.data());
  }
}

template <class T, typename Scalar, int POSE_SIZE>
const std::vector<size_t>&
LandmarkBlockBase<T, Scalar, POSE_SIZE>::get_pose_idx() const {
  return static_cast<const T*>(this)->get_pose_idx();
}

}  // namespace rootba
