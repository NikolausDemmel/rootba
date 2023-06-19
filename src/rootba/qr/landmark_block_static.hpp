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

template <typename Scalar, int POSE_SIZE, int NUM_OBS>
class LandmarkBlockStatic
    : public LandmarkBlockBase<LandmarkBlockStatic<Scalar, POSE_SIZE, NUM_OBS>,
                               Scalar, POSE_SIZE> {
 public:
  constexpr static size_t NUM_ROWS =
      NUM_OBS * 2 + 3;  // residuals and lm damping

  constexpr static size_t PADDING_IDX = NUM_OBS * POSE_SIZE;
  constexpr static size_t PADDING_SIZE =
      (PADDING_IDX % 4) == 0 ? 0 : 4 - (PADDING_IDX % 4);
  constexpr static size_t LM_IDX = PADDING_IDX + PADDING_SIZE;
  constexpr static size_t RES_IDX = LM_IDX + 3;
  constexpr static size_t NUM_COLS = RES_IDX + 1;

  using Options = typename LandmarkBlock<Scalar>::Options;
  using State = typename LandmarkBlock<Scalar>::State;
  using Landmark = typename BalProblem<Scalar>::Landmark;

  void allocate_landmark_impl(Landmark& lm) {
    BASALT_ASSERT(lm.obs.size() == NUM_OBS);
    pose_idx_.resize(NUM_OBS);

    size_t i = 0;
    for (const auto& [cam_idx, obs] : lm.obs) {
      pose_idx_[i] = cam_idx;
      i++;
    }
  }

  inline auto& get_storage() { return storage_; }
  inline const auto& get_storage() const { return storage_; }

  inline const std::vector<size_t>& get_pose_idx() const override {
    return pose_idx_;
  }
  inline size_t get_padding_idx() const { return PADDING_IDX; }
  inline size_t get_padding_size() const { return PADDING_SIZE; }
  inline size_t get_lm_idx() const { return LM_IDX; }
  inline size_t get_res_idx() const { return RES_IDX; }

  inline size_t get_num_cols() const { return NUM_COLS; }
  inline size_t get_num_rows() const { return NUM_ROWS; }

 protected:
  // Dense storage for pose Jacobians, padding, landmark Jacobians and
  // residuals [J_p | pad | J_l | res]
  Eigen::Matrix<Scalar, NUM_ROWS, NUM_COLS, Eigen::RowMajor> storage_;

  std::vector<size_t> pose_idx_;
};

}  // namespace rootba
