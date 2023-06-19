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

#include <tbb/parallel_for.h>

#include "rootba/cg/block_sparse_matrix.hpp"
#include "rootba/cg/utils.hpp"
#include "rootba/sc/landmark_block.hpp"
#include "rootba/util/assert.hpp"

namespace rootba {

template <typename Scalar>
class DiagonalPreconditioner : public Preconditioner<Scalar> {
 public:
  using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  DiagonalPreconditioner(const VecX& diag) {
    invdiag.resize(diag.size());

    for (int i = 0; i < diag.size(); i++) {
      if (diag(i) != Scalar(0)) {
        invdiag(i) = Scalar(1) / diag(i);
      } else {
        invdiag(i) = Scalar(1);
      }
    }
  }

  void solve_assign(const VecX& b, VecX& x) const override {
    x = invdiag.array() * b.array();
  }

  VecX invdiag;
};

template <typename Scalar>
class BlockDiagonalPreconditioner : public Preconditioner<Scalar> {
 public:
  using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
  using MatX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
  using RowMatX =
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  BlockDiagonalPreconditioner(int num_cams, int pose_size,
                              const IndexedBlocks<Scalar>& block_diag,
                              const VecX* diagonal)
      : num_cams(num_cams), pose_size(pose_size) {
    if (diagonal) {
      ROOTBA_ASSERT(diagonal->size() == num_cams * pose_size);
    }

    // allocate dense storage for all blocks
    inv_blocks.resize(num_rows(), pose_size);

    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t i = range.begin(); i != range.end(); ++i) {
        ROOTBA_ASSERT_MSG(block_diag.count(std::make_pair(i, i)),
                          "block diagonal is missing blocks");
        // NOTE: is doing a hash table lookup really the right thing here? There
        // is no blocked-range for std::unordered_map. Probably this is still
        // faster than creating a vector of iterators or moving the blocks into
        // a vector first.
        const auto& input_block = block_diag.at(std::make_pair(i, i));
        ROOTBA_ASSERT(pose_size == input_block.rows());
        ROOTBA_ASSERT(pose_size == input_block.cols());
        auto output_block =
            inv_blocks.block(i * pose_size, 0, pose_size, pose_size);
        if (diagonal) {
          MatX tmp = input_block;
          tmp +=
              diagonal->template segment(i * pose_size, pose_size).asDiagonal();
          output_block.noalias() =
              tmp.template selfadjointView<Eigen::Upper>().llt().solve(
                  MatX::Identity(pose_size, pose_size));
        } else {
          output_block.noalias() =
              input_block.template selfadjointView<Eigen::Upper>().llt().solve(
                  MatX::Identity(pose_size, pose_size));
        }
      }
    };

    tbb::blocked_range<size_t> range(0, unsigned_cast(num_cams));
    tbb::parallel_for(range, body);
  }

  void solve_assign(const VecX& b, VecX& x) const override {
    ROOTBA_ASSERT(b.size() == num_rows());
    ROOTBA_ASSERT(x.size() == num_rows());

    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t i = range.begin(); i != range.end(); ++i) {
        int idx = pose_size * i;
        auto block = inv_blocks.block(idx, 0, pose_size, pose_size);
        x.segment(idx, pose_size).noalias() = block * b.segment(idx, pose_size);
      }
    };

    tbb::blocked_range<size_t> range(0, unsigned_cast(num_cams));
    tbb::parallel_for(range, body);
  }

  int num_rows() const { return num_cams * pose_size; }

  int num_cams;
  int pose_size;
  RowMatX inv_blocks;
};

template <typename Scalar, int POSE_SIZE>
class PowerSCPreconditioner : public Preconditioner<Scalar> {
 public:
  using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
  using MatX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
  using RowMatX =
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  using LandmarkBlock = LandmarkBlockSC<Scalar, POSE_SIZE>;

  PowerSCPreconditioner(const size_t order, const IndexedBlocks<Scalar>& Hpp,
                        const std::vector<LandmarkBlock>& lm_blocks,
                        std::vector<std::mutex>& pose_mutex)
      : num_cams_(pose_mutex.size()),
        order_(order),
        lm_blocks_(lm_blocks),
        pose_mutex_(pose_mutex) {
    // Invert Hpp (should be already damped)
    Hpp_inv_.resize(POSE_SIZE * num_cams_, POSE_SIZE);
    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        ROOTBA_ASSERT_MSG(Hpp.count(std::make_pair(r, r)),
                          "Missing Hpp blocks");

        const auto& input_block = Hpp.at(std::make_pair(r, r));
        auto inv_block =
            Hpp_inv_.template block<POSE_SIZE, POSE_SIZE>(POSE_SIZE * r, 0);
        inv_block.noalias() =
            input_block.template selfadjointView<Eigen::Upper>().llt().solve(
                MatX::Identity(POSE_SIZE, POSE_SIZE));
      }
    };
    tbb::blocked_range<size_t> range(0, num_cams_);
    tbb::parallel_for(range, body);
  }

  void solve_assign(const VecX& b, VecX& x) const override {
    ROOTBA_ASSERT(b.size() == num_rows());
    ROOTBA_ASSERT(x.size() == num_rows());

    // The computations here are equivalent to the inner iterations of
    // LinearizationPowerSC solver.
    x = right_mul_Hpp_inv(b);
    VecX tmp = x;

    for (size_t i = 1; i <= order_; ++i) {
      tmp = right_mul_Hpp_inv(right_mul_e0(tmp));
      x += tmp;
    }
  }

  int num_rows() const { return num_cams_ * POSE_SIZE; }

  // For debugging only
  inline auto get_Hpp_inv(const size_t cam_idx) const {
    return Hpp_inv_.template block<POSE_SIZE, POSE_SIZE>(POSE_SIZE * cam_idx,
                                                         0);
  }

 protected:
  inline VecX right_mul_Hpp_inv(const VecX& x) const {
    ROOTBA_ASSERT(static_cast<int>(x.size()) == num_cams_ * POSE_SIZE);

    VecX res(num_cams_ * POSE_SIZE);
    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        const auto u =
            Hpp_inv_.template block<POSE_SIZE, POSE_SIZE>(POSE_SIZE * r, 0);
        const auto v = x.template segment<POSE_SIZE>(POSE_SIZE * r);
        res.template segment<POSE_SIZE>(POSE_SIZE * r) = u * v;
      }
    };

    tbb::blocked_range<size_t> range(0, num_cams_);
    tbb::parallel_for(range, body);

    return res;
  }

  inline VecX right_mul_e0(const VecX& x) const {
    ROOTBA_ASSERT(static_cast<int>(x.size()) == num_cams_ * POSE_SIZE);

    VecX res = VecX::Zero(num_cams_ * POSE_SIZE);
    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        const auto& lb = lm_blocks_[r];

        VecX Jp_x(2 * lb.num_poses());
        lb.add_Jp_x(Jp_x, x);

        const auto Jl = lb.get_Jl();
        const VecX tmp = Jl * (lb.get_Hll_inv() * (Jl.transpose() * Jp_x));

        lb.add_JpT_x(res, tmp, &pose_mutex_);
      }
    };

    tbb::blocked_range<size_t> range(0, lm_blocks_.size());
    tbb::parallel_for(range, body);

    return res;
  }

  const int num_cams_;
  const size_t order_;

  const std::vector<LandmarkBlock>& lm_blocks_;
  std::vector<std::mutex>& pose_mutex_;

  RowMatX Hpp_inv_;
};

}  // namespace rootba
