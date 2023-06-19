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

#include <mutex>

#include <Eigen/Dense>
#include <glog/logging.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

#include "rootba/bal/bal_problem.hpp"
#include "rootba/qr/linearization_utils.hpp"
#include "rootba/sc/landmark_block.hpp"
#include "rootba/util/assert.hpp"
#include "rootba/util/cast.hpp"
#include "rootba/util/format.hpp"

namespace rootba {

template <typename Scalar_, int POSE_SIZE_>
class LinearizationSC {
 public:
  using Scalar = Scalar_;
  static constexpr int POSE_SIZE = POSE_SIZE_;

  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
  using Vec4 = Eigen::Matrix<Scalar, 4, 1>;

  using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
  using MatX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

  using Mat36 = Eigen::Matrix<Scalar, 3, 6>;

  using Options = typename LandmarkBlockSC<Scalar, POSE_SIZE>::Options;

  LinearizationSC(BalProblem<Scalar>& bal_problem, const Options& options,
                  const bool enable_reduction_alg = true)
      : options_(options),
        bal_problem_(bal_problem),
        num_cameras_(bal_problem_.cameras().size()),
        pose_mutex_(num_cameras_) {
    const size_t num_landmarks = bal_problem_.landmarks().size();
    landmark_blocks_.resize(num_landmarks);

    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        landmark_blocks_[r].allocate_landmark(bal_problem_.landmarks()[r],
                                              options_);
      }
    };

    tbb::blocked_range<size_t> range(0, num_landmarks);
    tbb::parallel_for(range, body);

    // H_pp_mutex_ won't be allocated when this class is inherited,
    // only used in get_Hb
    if (enable_reduction_alg && options_.reduction_alg == 1) {
      H_pp_mutex_ = std::vector<std::mutex>(num_cameras_ * num_cameras_);
    }
  };

  // return value `false` indicates numerical failure --> linearization at this
  // state is unusable. Numeric check is only performed for residuals that were
  // considered to be used (valid), which depends on use_valid_projections_only
  // setting.
  bool linearize_problem() {
    ROOTBA_ASSERT(bal_problem_.landmarks().size() == landmark_blocks_.size());

    size_t num_landmarks = landmark_blocks_.size();

    auto body = [&](const tbb::blocked_range<size_t>& range,
                    bool numerically_valid) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        landmark_blocks_[r].linearize_landmark(bal_problem_.cameras());
        numerically_valid &= !landmark_blocks_[r].is_numerical_failure();
      }
      return numerically_valid;
    };

    tbb::blocked_range<size_t> range(0, num_landmarks);
    const bool numerically_valid =
        tbb::parallel_reduce(range, true, body, std::logical_and<>());

    return numerically_valid;
  };

  size_t num_cols_reduced() const { return num_cameras_ * POSE_SIZE; }

  Scalar back_substitute(const VecX& pose_inc) {
    ROOTBA_ASSERT(pose_inc.size() == signed_cast(num_cameras_ * POSE_SIZE));

    auto body = [&](const tbb::blocked_range<size_t>& range, Scalar l_diff) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        landmark_blocks_[r].back_substitute(pose_inc, l_diff);
      }
      return l_diff;
    };

    tbb::blocked_range<size_t> range(0, landmark_blocks_.size());
    Scalar l_diff =
        tbb::parallel_reduce(range, Scalar(0), body, std::plus<Scalar>());
    return l_diff;
  }

  VecX get_Jp_diag2() const {
    struct Reductor {
      Reductor(size_t num_rows,
               const std::vector<LandmarkBlockSC<Scalar, POSE_SIZE>>&
                   landmark_blocks)
          : num_rows(num_rows), landmark_blocks(landmark_blocks) {
        res.setZero(num_rows);
      }

      void operator()(const tbb::blocked_range<size_t>& range) {
        for (size_t r = range.begin(); r != range.end(); ++r) {
          const auto& lb = landmark_blocks[r];
          lb.add_Jp_diag2(res);
        }
      }

      Reductor(Reductor& a, tbb::split /*unused*/)
          : num_rows(a.num_rows), landmark_blocks(a.landmark_blocks) {
        res.setZero(num_rows);
      };

      inline void join(const Reductor& b) { res += b.res; }

      size_t num_rows;
      const std::vector<LandmarkBlockSC<Scalar, POSE_SIZE>>& landmark_blocks;
      VecX res;
    };

    Reductor r(num_cameras_ * POSE_SIZE, landmark_blocks_);

    tbb::blocked_range<size_t> range(0, landmark_blocks_.size());
    tbb::parallel_reduce(range, r);

    // TODO: double check including vs not including pose damping here in usage
    // and make it clear in API; see also getJpTJp_blockdiag

    // Note: ignore damping here

    return r.res;
  }

  void scale_Jl_cols() {
    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        landmark_blocks_[r].scale_Jl_cols();
      }
    };

    tbb::blocked_range<size_t> range(0, landmark_blocks_.size());
    tbb::parallel_for(range, body);
  }

  void scale_Jp_cols(const VecX& jacobian_scaling) {
    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        landmark_blocks_[r].scale_Jp_cols(jacobian_scaling);
      }
    };

    tbb::blocked_range<size_t> range(0, landmark_blocks_.size());
    tbb::parallel_for(range, body);
  }

  /// compute scale and apply scaling in one go, e.g. for unit tests.
  void compute_Jp_scale_and_scale_Jp_cols() {
    const VecX Jp_scale2 = get_Jp_diag2();
    const VecX Jp_scaling =
        compute_jacobi_scaling(Jp_scale2, options_.jacobi_scaling_eps);
    scale_Jp_cols(Jp_scaling);
  }

  void set_pose_damping(const Scalar lambda) {
    ROOTBA_ASSERT(lambda >= 0);

    pose_damping_diagonal_ = lambda;
  }

  inline bool has_pose_damping() const { return pose_damping_diagonal_ > 0; }

  void set_landmark_damping(const Scalar lambda) {
    ROOTBA_ASSERT(lambda >= 0);

    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        landmark_blocks_[r].set_landmark_damping(lambda);
      }
    };

    tbb::blocked_range<size_t> range(0, landmark_blocks_.size());
    tbb::parallel_for(range, body);
  }

  void get_Hb(BlockSparseMatrix<Scalar>& H_pp, VecX& b_p) const {
    if (options_.reduction_alg == 0) {
      get_hb_r(H_pp, b_p);
    } else if (options_.reduction_alg == 1) {
      get_hb_f(H_pp, b_p);
    } else {
      LOG(FATAL) << "options_.reduction_alg " << options_.reduction_alg
                 << " is not supported.";
    }
    H_pp.recompute_keys();
  }

  BlockDiagonalAccumulator<Scalar> get_jacobi() const {
    BlockDiagonalAccumulator<Scalar> accum;
    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        const auto& lb = landmark_blocks_[r];
        lb.add_Hpp(accum, pose_mutex_);
      }
    };
    tbb::blocked_range<size_t> range(0, landmark_blocks_.size());
    tbb::parallel_for(range, body);

    if (has_pose_damping()) {
      accum.add_diag(
          num_cameras_, POSE_SIZE,
          VecX::Constant(num_cameras_ * POSE_SIZE, pose_damping_diagonal_));
    }

    // TODO: double check including vs not including pose damping here in usage
    // and make it clear in API; see also getJp_diag2
    return accum;
  }

  const auto& get_landmark_blocks() const { return landmark_blocks_; }

  auto& get_pose_mutex() const { return pose_mutex_; }

  void print_block(const std::string& filename, size_t block_idx) {
    landmark_blocks_[block_idx].printStorage(filename);
  }

 protected:
  void get_hb_r(BlockSparseMatrix<Scalar>& H_pp, VecX& b_p) const {
    struct Reductor {
      Reductor(const std::vector<LandmarkBlockSC<Scalar, POSE_SIZE>>&
                   landmark_blocks,
               size_t num_cameras)
          : landmark_blocks(landmark_blocks),
            num_cameras(num_cameras),
            H_pp(num_cameras * POSE_SIZE, num_cameras * POSE_SIZE) {
        b_p.setZero(num_cameras * POSE_SIZE);
      }

      Reductor(Reductor& a, tbb::split /*unused*/)
          : landmark_blocks(a.landmark_blocks),
            num_cameras(a.num_cameras),
            H_pp(num_cameras * POSE_SIZE, num_cameras * POSE_SIZE) {
        b_p.setZero(a.num_cameras * POSE_SIZE);
      }

      void operator()(const tbb::blocked_range<size_t>& range) {
        for (size_t r = range.begin(); r != range.end(); ++r) {
          const auto& lb = landmark_blocks[r];
          lb.add_Hb(H_pp, b_p);
        }
      }

      inline void join(Reductor& other) {
        H_pp.join(other.H_pp);
        b_p += other.b_p;
      }

      const std::vector<LandmarkBlockSC<Scalar, POSE_SIZE>>& landmark_blocks;
      size_t num_cameras;

      BlockSparseMatrix<Scalar> H_pp;
      VecX b_p;
    };

    Reductor r(landmark_blocks_, num_cameras_);

    tbb::blocked_range<size_t> range(0, landmark_blocks_.size());
    tbb::parallel_reduce(range, r);

    if (has_pose_damping()) {
      r.H_pp.add_diag(
          num_cameras_, POSE_SIZE,
          VecX::Constant(num_cameras_ * POSE_SIZE, pose_damping_diagonal_));
    }

    H_pp = std::move(r.H_pp);
    b_p = std::move(r.b_p);
  }

  void get_hb_f(BlockSparseMatrix<Scalar>& H_pp, VecX& b_p) const {
    ROOTBA_ASSERT(H_pp_mutex_.size() == num_cameras_ * num_cameras_);

    // Fill H_pp and b_p
    b_p.setZero(H_pp.rows);
    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        const auto& lb = landmark_blocks_[r];
        lb.add_Hb(H_pp, b_p, H_pp_mutex_, pose_mutex_, num_cameras_);
      }
    };

    tbb::blocked_range<size_t> range(0, landmark_blocks_.size());
    tbb::parallel_for(range, body);

    if (has_pose_damping()) {
      H_pp.add_diag(
          num_cameras_, POSE_SIZE,
          VecX::Constant(num_cameras_ * POSE_SIZE, pose_damping_diagonal_));
    }
  }

  Options options_;
  BalProblem<Scalar>& bal_problem_;
  size_t num_cameras_;

  mutable std::vector<std::mutex> H_pp_mutex_;
  mutable std::vector<std::mutex> pose_mutex_;

  std::vector<LandmarkBlockSC<Scalar, POSE_SIZE>> landmark_blocks_;

  Scalar pose_damping_diagonal_ = 0;
};

}  // namespace rootba
