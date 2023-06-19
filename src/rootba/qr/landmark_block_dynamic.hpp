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

#include "rootba/qr/landmark_block_base.hpp"

namespace rootba {

template <typename Scalar, int POSE_SIZE>
class LandmarkBlockDynamic
    : public LandmarkBlockBase<LandmarkBlockDynamic<Scalar, POSE_SIZE>, Scalar,
                               POSE_SIZE> {
 public:
  using Landmark = typename BalProblem<Scalar>::Landmark;

  inline void allocate_landmark_impl(Landmark& lm) {
    pose_idx_.clear();
    pose_idx_.reserve(lm.obs.size());
    for (const auto& [cam_idx, obs] : lm.obs) {
      pose_idx_.push_back(cam_idx);
    }

    padding_idx_ = pose_idx_.size() * POSE_SIZE;
    num_rows_ = pose_idx_.size() * 2 + 3;  // residuals and lm damping

    size_t pad = padding_idx_ % 4;
    if (pad != 0) {
      padding_size_ = 4 - pad;
    }

    lm_idx_ = padding_idx_ + padding_size_;
    res_idx_ = lm_idx_ + 3;
    num_cols_ = res_idx_ + 1;

    storage_.resize(num_rows_, num_cols_);
  }

  inline auto& get_storage() { return storage_; }
  inline const auto& get_storage() const { return storage_; }

  inline const std::vector<size_t>& get_pose_idx() const override {
    return pose_idx_;
  }
  inline size_t get_padding_idx() const { return padding_idx_; }
  inline size_t get_padding_size() const { return padding_size_; }
  inline size_t get_lm_idx() const { return lm_idx_; }
  inline size_t get_res_idx() const { return res_idx_; }

  inline size_t get_num_cols() const { return num_cols_; }
  inline size_t get_num_rows() const { return num_rows_; }

 protected:
  // Dense storage for pose Jacobians, padding, landmark Jacobians and
  // residuals [J_p | pad | J_l | res]
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      storage_;

  std::vector<size_t> pose_idx_;
  size_t padding_idx_ = 0;
  size_t padding_size_ = 0;
  size_t lm_idx_ = 0;
  size_t res_idx_ = 0;

  size_t num_cols_ = 0;
  size_t num_rows_ = 0;
};

}  // namespace rootba
