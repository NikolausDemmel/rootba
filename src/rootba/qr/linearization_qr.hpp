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
#include "rootba/qr/landmark_block.hpp"
#include "rootba/util/assert.hpp"
#include "rootba/util/cast.hpp"
#include "rootba/util/format.hpp"

namespace rootba {

template <typename Scalar_, int POSE_SIZE_>
class LinearizationQR : public LinearOperator<Scalar_> {
 public:
  using Scalar = Scalar_;
  static constexpr int POSE_SIZE = POSE_SIZE_;

  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
  using Vec4 = Eigen::Matrix<Scalar, 4, 1>;
  using VecP = Eigen::Matrix<Scalar, POSE_SIZE, 1>;

  using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  using Mat36 = Eigen::Matrix<Scalar, 3, 6>;

  using LandmarkBlockPtr = std::unique_ptr<LandmarkBlock<Scalar>>;

  // TODO@demmeln: consider removing this struct and moving the extra contents
  // into LandmarkBlock::Options (like for SC), or better, create a separate
  // Options class outside of the LandmarkBlock class to reduce include
  // dependencies and better separate the concerns. This can also simplify some
  // options assignment code.
  struct Options {
    int reduction_alg = 1;
    typename LandmarkBlock<Scalar>::Options lb_options;
  };

  LinearizationQR(BalProblem<Scalar>& bal_problem, const Options& options)
      : options_(options), bal_problem_(bal_problem) {
    size_t num_lms = bal_problem_.landmarks().size();

    landmark_blocks_.resize(num_lms);

    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        auto& lb = landmark_blocks_[r];
        auto& landmark = bal_problem_.landmarks()[r];

        lb = LandmarkBlockFactory<Scalar, POSE_SIZE>::get_landmark_block(
            landmark.obs.size());

        lb->allocate_landmark(landmark, options.lb_options);
      }
    };

    tbb::blocked_range<size_t> range(0, num_lms);
    tbb::parallel_for(range, body);

    landmark_block_idx_.reserve(num_lms);

    num_rows_Q2Tr_ = 0;
    for (size_t i = 0; i < num_lms; i++) {
      landmark_block_idx_.emplace_back(num_rows_Q2Tr_);
      num_rows_Q2Tr_ += landmark_blocks_[i]->num_Q2T_rows();
    }

    num_cameras_ = bal_problem_.cameras().size();
    std::vector<std::mutex>(num_cameras_).swap(pose_mutex_);
  }

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
        landmark_blocks_[r]->linearize_landmark(bal_problem_.cameras());
        numerically_valid &= !landmark_blocks_[r]->is_numerical_failure();
      }
      return numerically_valid;
    };

    tbb::blocked_range<size_t> range(0, num_landmarks);
    const bool numerically_valid =
        tbb::parallel_reduce(range, true, body, std::logical_and<>());

    return numerically_valid;
  }

  void set_pose_damping(const Scalar lambda) {
    ROOTBA_ASSERT(lambda >= 0);

    pose_damping_diagonal_ = lambda;
    pose_damping_diagonal_sqrt_ = std::sqrt(lambda);
  }

  bool has_pose_damping() const { return pose_damping_diagonal_ > 0; }

  size_t num_rows_reduced() const {
    return has_pose_damping() ? num_rows_Q2Tr_ + num_cameras_ * POSE_SIZE
                              : num_rows_Q2Tr_;
  }

  size_t num_cols_reduced() const { return num_cameras_ * POSE_SIZE; }

  void perform_qr() {
    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        landmark_blocks_[r]->perform_qr();
      }
    };

    tbb::blocked_range<size_t> range(0, landmark_block_idx_.size());
    tbb::parallel_for(range, body);
  }

  Scalar back_substitute(const VecX& pose_inc) {
    ROOTBA_ASSERT(pose_inc.size() == signed_cast(num_cameras_ * POSE_SIZE));

    auto body = [&](const tbb::blocked_range<size_t>& range, Scalar l_diff) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        landmark_blocks_[r]->back_substitute(pose_inc, l_diff);
      }
      return l_diff;
    };

    tbb::blocked_range<size_t> range(0, landmark_block_idx_.size());
    Scalar l_diff =
        tbb::parallel_reduce(range, Scalar(0), body, std::plus<Scalar>());
    return l_diff;
  }

  Eigen::SparseMatrix<Scalar, Eigen::RowMajor> get_Q2TJp() const {
    // Since we know exactly the sparsity, we can do better than triplets:
    // https://stackoverflow.com/a/18160211/1813258
    // https://eigen.tuxfamily.org/dox/group__SparseQuickRefPage.html
    // But since it's for debugging, it probably doesn't matter...

    // prepare triplet vector
    size_t num_triplets = 0;
    for (const auto& lb : landmark_blocks_) {
      num_triplets += lb->num_reduced_cams() * POSE_SIZE * lb->num_Q2T_rows();
    }
    if (has_pose_damping()) {
      num_triplets += num_cameras_ * POSE_SIZE;
    }
    std::vector<Eigen::Triplet<Scalar>> triplets;
    triplets.reserve(num_triplets);

    // generate all triplets
    for (size_t i = 0; i < landmark_blocks_.size(); ++i) {
      const auto& lb = landmark_blocks_[i];
      lb->add_triplets_Q2TJp(landmark_block_idx_[i], triplets);
    }

    // add damping entries
    if (has_pose_damping()) {
      for (size_t i = 0; i < num_cameras_ * POSE_SIZE; ++i) {
        triplets.emplace_back(num_rows_Q2Tr_ + i, i,
                              pose_damping_diagonal_sqrt_);
      }
    }

    // build sparse matrix
    Eigen::SparseMatrix<Scalar, Eigen::RowMajor> res(num_rows_reduced(),
                                                     POSE_SIZE * num_cameras_);
    if (!triplets.empty()) {
      res.setFromTriplets(triplets.begin(), triplets.end());
    }

    return res;
  }

  VecX get_Q2TJp_postmult_x(const VecX& x_pose) const {
    ROOTBA_ASSERT(x_pose.size() == signed_cast(num_cameras_ * POSE_SIZE));

    VecX res(num_rows_reduced());

    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        res.segment(landmark_block_idx_[r],
                    landmark_blocks_[r]->num_Q2T_rows()) =
            landmark_blocks_[r]->get_Q2TJp_postmult_x(x_pose);
      }
    };

    tbb::blocked_range<size_t> range(0, landmark_block_idx_.size());
    tbb::parallel_for(range, body);

    if (has_pose_damping()) {
      res.tail(POSE_SIZE * num_cameras_) =
          x_pose.array() * pose_damping_diagonal_sqrt_;
    }

    return res;
  }

  VecX get_Q2TJp_premult_x(const VecX& x_r) const {
    ROOTBA_ASSERT(x_r.size() == signed_cast(num_rows_reduced()));

    auto body = [&](const tbb::blocked_range<size_t>& range, VecX res) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        const auto& lb = landmark_blocks_[r];

        lb->add_Q2TJp_premult_x(
            res, x_r.segment(landmark_block_idx_[r], lb->num_Q2T_rows()));
      }
      return res;
    };

    auto join = [](const auto& x, const auto& y) { return x + y; };

    VecX init;
    init.setZero(POSE_SIZE * num_cameras_);

    // go over all host frames
    tbb::blocked_range<size_t> range(0, landmark_block_idx_.size());
    VecX res = tbb::parallel_reduce(range, init, body, join);

    if (has_pose_damping()) {
      res += (x_r.tail(POSE_SIZE * num_cameras_).array() *
              pose_damping_diagonal_sqrt_)
                 .matrix();
    }

    return res;
  }

  inline VecX get_Q2TJp_T_Q2TJp_mult_x(const VecX& x_pose) const {
    switch (options_.reduction_alg) {
      case 0:
        return get_Q2TJp_T_Q2TJp_mult_x_v0(x_pose);
        //      case 1:
        //        return getQ2JpTQ2Jp_mult_x_v1(x_pose);
        //      case 2:
        //        return getQ2JpTQ2Jp_mult_x_v2(x_pose);
      case 1:
        return get_Q2TJp_T_Q2TJp_mult_x_v3(x_pose);

      default:
        LOG(FATAL) << "options_.reduction_alg " << options_.reduction_alg
                   << " is not supported.";
    }
  }

  VecX get_Q2TJp_T_Q2TJp_mult_x_v0(const VecX& x_pose) const {
    ROOTBA_ASSERT(x_pose.size() == signed_cast(num_cameras_ * POSE_SIZE));

    struct Reductor {
      Reductor(const VecX& x_pose,
               const std::vector<LandmarkBlockPtr>& landmark_blocks)
          : x_pose(x_pose), landmark_blocks(landmark_blocks) {
        res.setZero(x_pose.rows());
      }

      void operator()(const tbb::blocked_range<size_t>& range) {
        for (size_t r = range.begin(); r != range.end(); ++r) {
          const auto& lb = landmark_blocks[r];
          lb->add_Q2TJp_T_Q2TJp_mult_x(res, x_pose);
        }
      }

      Reductor(Reductor& a, tbb::split /*unused*/)
          : x_pose(a.x_pose), landmark_blocks(a.landmark_blocks) {
        res.setZero(x_pose.rows());
      }

      inline void join(const Reductor& b) { res += b.res; }

      const VecX& x_pose;
      const std::vector<LandmarkBlockPtr>& landmark_blocks;
      VecX res;
    };

    Reductor r(x_pose, landmark_blocks_);

    // go over all landmarks
    tbb::blocked_range<size_t> range(0, landmark_block_idx_.size());
    tbb::parallel_reduce(range, r);

    if (has_pose_damping()) {
      r.res += (x_pose.array() * pose_damping_diagonal_).matrix();
    }

    return r.res;
  }

  // Version with unordered_map and mutex guarded writes
  VecX get_Q2TJp_T_Q2TJp_mult_x_v1(const VecX& x_pose) const {
    ROOTBA_ASSERT(x_pose.size() == signed_cast(num_cameras_ * POSE_SIZE));

    VecX res;
    res.setZero(x_pose.rows());

    auto body = [&](const tbb::blocked_range<size_t>& range) {
      std::unordered_map<int, VecP> partial_sum_map;
      partial_sum_map.reserve(num_cameras_ / 8);

      for (size_t r = range.begin(); r != range.end(); ++r) {
        const auto& lb = landmark_blocks_[r];
        // lb->mapQ2JpTQ2Jp_mult_x(partial_sum_map, x_pose);
      }

      for (const auto& [k, v] : partial_sum_map) {
        std::scoped_lock<std::mutex> lock(pose_mutex_[k]);
        res.template segment<POSE_SIZE>(k * POSE_SIZE) += v;
      }

      return res;
    };

    // go over all host frames
    tbb::blocked_range<size_t> range(0, landmark_block_idx_.size());
    tbb::parallel_for(range, body);

    if (has_pose_damping()) {
      res += (x_pose.array() * pose_damping_diagonal_).matrix();
    }

    return res;
  }

  // Verstion with map and mutex guarded writes
  VecX get_Q2TJp_T_Q2TJp_mult_x_v2(const VecX& x_pose) const {
    ROOTBA_ASSERT(x_pose.size() == signed_cast(num_cameras_ * POSE_SIZE));

    VecX res;
    res.setZero(x_pose.rows());

    auto body = [&](const tbb::blocked_range<size_t>& range) {
      std::map<int, VecP> partial_sum_map;

      for (size_t r = range.begin(); r != range.end(); ++r) {
        const auto& lb = landmark_blocks_[r];
        // lb->mapQ2JpTQ2Jp_mult_x(partial_sum_map, x_pose);
      }

      for (const auto& [k, v] : partial_sum_map) {
        std::scoped_lock<std::mutex> lock(pose_mutex_[k]);
        res.template segment<POSE_SIZE>(k * POSE_SIZE) += v;
      }

      return res;
    };

    // go over all host frames
    tbb::blocked_range<size_t> range(0, landmark_block_idx_.size());
    tbb::parallel_for(range, body);

    if (has_pose_damping()) {
      res += (x_pose.array() * pose_damping_diagonal_).matrix();
    }

    return res;
  }

  // Verstion with direct mutex guarded writes
  VecX get_Q2TJp_T_Q2TJp_mult_x_v3(const VecX& x_pose) const {
    ROOTBA_ASSERT(x_pose.size() == signed_cast(num_cameras_ * POSE_SIZE));

    VecX res;
    res.setZero(x_pose.rows());

    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        const auto& lb = landmark_blocks_[r];
        lb->add_Q2TJp_T_Q2TJp_mult_x(res, x_pose, &pose_mutex_);
      }

      return res;
    };

    tbb::blocked_range<size_t> range(0, landmark_block_idx_.size());
    tbb::parallel_for(range, body);

    if (has_pose_damping()) {
      res += (x_pose.array() * pose_damping_diagonal_).matrix();
    }

    return res;
  }

  VecX get_Q2TJp_T_Q2Tr() const {
    auto body = [&](const tbb::blocked_range<size_t>& range, VecX res) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        const auto& lb = landmark_blocks_[r];
        lb->add_Q2TJp_T_Q2Tr(res);
      }
      return res;
    };

    auto join = [](const auto& x, const auto& y) { return x + y; };

    VecX init;
    init.setZero(POSE_SIZE * num_cameras_);

    tbb::blocked_range<size_t> range(0, landmark_block_idx_.size());
    VecX res = tbb::parallel_reduce(range, init, body, join);

    // Note: No need to consider pose damping, since the residual part is 0

    return res;
  }

  VecX get_Q2TJp_diag2() const {
    struct Reductor {
      Reductor(size_t num_rows,
               const std::vector<LandmarkBlockPtr>& landmark_blocks)
          : num_rows(num_rows), landmark_blocks(landmark_blocks) {
        res.setZero(num_rows);
      }

      void operator()(const tbb::blocked_range<size_t>& range) {
        for (size_t r = range.begin(); r != range.end(); ++r) {
          const auto& lb = landmark_blocks[r];
          lb->add_Q2TJp_diag2(res);
        }
      }

      Reductor(Reductor& a, tbb::split /*unused*/)
          : num_rows(a.num_rows), landmark_blocks(a.landmark_blocks) {
        res.setZero(num_rows);
      }

      inline void join(const Reductor& b) { res += b.res; }

      size_t num_rows;
      const std::vector<LandmarkBlockPtr>& landmark_blocks;
      VecX res;
    };

    Reductor r(num_cameras_ * POSE_SIZE, landmark_blocks_);

    tbb::blocked_range<size_t> range(0, landmark_block_idx_.size());
    tbb::parallel_reduce(range, r);

    if (has_pose_damping()) {
      r.res.array() += pose_damping_diagonal_;
    }

    return r.res;
  }

  VecX get_Jp_diag2() const {
    struct Reductor {
      Reductor(size_t num_rows,
               const std::vector<LandmarkBlockPtr>& landmark_blocks)
          : num_rows(num_rows), landmark_blocks(landmark_blocks) {
        res.setZero(num_rows);
      }

      void operator()(const tbb::blocked_range<size_t>& range) {
        for (size_t r = range.begin(); r != range.end(); ++r) {
          const auto& lb = landmark_blocks[r];
          lb->add_Jp_diag2(res);
        }
      }

      Reductor(Reductor& a, tbb::split /*unused*/)
          : num_rows(a.num_rows), landmark_blocks(a.landmark_blocks) {
        res.setZero(num_rows);
      }

      inline void join(const Reductor& b) { res += b.res; }

      size_t num_rows;
      const std::vector<LandmarkBlockPtr>& landmark_blocks;
      VecX res;
    };

    Reductor r(num_cameras_ * POSE_SIZE, landmark_blocks_);

    tbb::blocked_range<size_t> range(0, landmark_block_idx_.size());
    tbb::parallel_reduce(range, r);

    // NOTE: no damping here (it's used for Jp column scaling)

    return r.res;
  }

  IndexedBlocks<Scalar> get_Q2TJp_T_Q2TJp_blockdiag() const {
    struct Reductor {
      Reductor(const std::vector<LandmarkBlockPtr>& landmark_blocks)
          : landmark_blocks(landmark_blocks) {}

      Reductor(Reductor& a, tbb::split /*unused*/)
          : landmark_blocks(a.landmark_blocks) {}

      void operator()(const tbb::blocked_range<size_t>& range) {
        for (size_t r = range.begin(); r != range.end(); ++r) {
          const auto& lb = landmark_blocks[r];
          lb->add_Q2TJp_T_Q2TJp_blockdiag(accum);
        }
      }

      inline void join(Reductor& b) { accum.join(b.accum); }

      BlockDiagonalAccumulator<Scalar> accum;
      const std::vector<LandmarkBlockPtr>& landmark_blocks;
    };

    Reductor r(landmark_blocks_);

    tbb::blocked_range<size_t> range(0, landmark_block_idx_.size());
    tbb::parallel_reduce(range, r);

    if (has_pose_damping()) {
      r.accum.add_diag(
          num_cameras_, POSE_SIZE,
          VecX::Constant(num_cameras_ * POSE_SIZE, pose_damping_diagonal_));
    }

    return r.accum.block_diagonal;
  }

  IndexedBlocks<Scalar> get_Jp_T_Jp_blockdiag() const {
    struct Reductor {
      Reductor(const std::vector<LandmarkBlockPtr>& landmark_blocks)
          : landmark_blocks(landmark_blocks) {}

      Reductor(Reductor& a, tbb::split /*unused*/)
          : landmark_blocks(a.landmark_blocks) {}

      void operator()(const tbb::blocked_range<size_t>& range) {
        for (size_t r = range.begin(); r != range.end(); ++r) {
          const auto& lb = landmark_blocks[r];
          lb->add_Jp_T_Jp_blockdiag(accum);
        }
      }

      inline void join(Reductor& b) { accum.join(b.accum); }

      BlockDiagonalAccumulator<Scalar> accum;
      const std::vector<LandmarkBlockPtr>& landmark_blocks;
    };

    Reductor r(landmark_blocks_);

    tbb::blocked_range<size_t> range(0, landmark_block_idx_.size());
    tbb::parallel_reduce(range, r);

    if (has_pose_damping()) {
      r.accum.add_diag(
          num_cameras_, POSE_SIZE,
          VecX::Constant(num_cameras_ * POSE_SIZE, pose_damping_diagonal_));
    }

    return r.accum.block_diagonal;
  }

  void scale_Jl_cols() {
    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        landmark_blocks_[r]->scale_Jl_cols();
      }
    };

    tbb::blocked_range<size_t> range(0, landmark_block_idx_.size());
    tbb::parallel_for(range, body);
  }

  void scale_Jp_cols(const VecX& jacobian_scaling) {
    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        landmark_blocks_[r]->scale_Jp_cols(jacobian_scaling);
      }
    };

    tbb::blocked_range<size_t> range(0, landmark_block_idx_.size());
    tbb::parallel_for(range, body);
  }

  void set_landmark_damping(Scalar lambda) {
    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        landmark_blocks_[r]->set_landmark_damping(lambda);
      }
    };

    tbb::blocked_range<size_t> range(0, landmark_block_idx_.size());
    tbb::parallel_for(range, body);
  }

  // Operations grouped into stages. Stage1 includes linearization, optional
  // JACOBI preconditioner, QR and column norms computations for LM damping.
  VecX get_stage1(IndexedBlocks<Scalar>* precond_block_diagonal) {
    struct Reductor {
      Reductor(size_t num_rows, bool precond_block_diagonal,
               std::vector<LandmarkBlockPtr>& landmark_blocks,
               const BalProblem<Scalar>& bal_problem)
          : num_rows(num_rows),
            landmark_blocks(landmark_blocks),
            bal_problem(bal_problem) {
        if (precond_block_diagonal) {
          precond_block_diagonal_accum.reset(
              new BlockDiagonalAccumulator<Scalar>());
        }
        res.setZero(num_rows);
      }

      void operator()(const tbb::blocked_range<size_t>& range) {
        for (size_t r = range.begin(); r != range.end(); ++r) {
          auto& lb = landmark_blocks[r];
          lb->linearize_landmark(bal_problem.cameras());
          if (!lb->is_numerical_failure()) {
            if (precond_block_diagonal_accum) {
              lb->add_Jp_T_Jp_blockdiag(*precond_block_diagonal_accum);
            }
            lb->add_Jp_diag2(res);
            lb->scale_Jl_cols();
            lb->perform_qr();
          } else {
            numerically_valid = false;
          }
        }
      }

      Reductor(Reductor& a, tbb::split /*unused*/)
          : num_rows(a.num_rows),
            landmark_blocks(a.landmark_blocks),
            bal_problem(a.bal_problem) {
        if (a.precond_block_diagonal_accum) {
          precond_block_diagonal_accum.reset(
              new BlockDiagonalAccumulator<Scalar>());
        }
        res.setZero(num_rows);
      }

      inline void join(Reductor& b) {
        if (precond_block_diagonal_accum) {
          precond_block_diagonal_accum->join(*b.precond_block_diagonal_accum);
        }
        res += b.res;
        numerically_valid &= b.numerically_valid;
      }

      size_t num_rows;
      std::vector<LandmarkBlockPtr>& landmark_blocks;
      const BalProblem<Scalar>& bal_problem;
      std::unique_ptr<BlockDiagonalAccumulator<Scalar>>
          precond_block_diagonal_accum;
      VecX res;
      bool numerically_valid = true;
    };

    const bool compute_precond_block_diagonal = precond_block_diagonal;

    Reductor r(num_cameras_ * POSE_SIZE, compute_precond_block_diagonal,
               landmark_blocks_, bal_problem_);

    tbb::blocked_range<size_t> range(0, landmark_block_idx_.size());
    tbb::parallel_reduce(range, r);

    if (r.numerically_valid) {
      if (compute_precond_block_diagonal) {
        *precond_block_diagonal =
            std::move(r.precond_block_diagonal_accum->block_diagonal);
      }

      return r.res;
    } else {
      return {};  // for now, return empty vector to indicate numerical failure
    }
  }

  // Operations grouped into stages. Stage2 includes Landmark damping, bref and
  // precondition computations.
  void get_stage2(Scalar lambda, const VecX* jacobian_scaling,
                  IndexedBlocks<Scalar>* precond_block_diagonal, VecX& bref) {
    struct Reductor {
      Reductor(size_t num_rows, Scalar lambda, const VecX* jacobian_scaling,
               bool precond_block_diagonal,
               std::vector<LandmarkBlockPtr>& landmark_blocks)
          : num_rows(num_rows),
            lambda(lambda),
            jacobian_scaling(jacobian_scaling),
            landmark_blocks(landmark_blocks) {
        if (precond_block_diagonal) {
          precond_block_diagonal_accum.reset(
              new BlockDiagonalAccumulator<Scalar>());
        }
        bref.setZero(num_rows);
      }

      void operator()(const tbb::blocked_range<size_t>& range) {
        for (size_t r = range.begin(); r != range.end(); ++r) {
          auto& lb = landmark_blocks[r];

          lb->stage2(lambda, jacobian_scaling,
                     precond_block_diagonal_accum.get(), bref);

          //          // 1. scale jacobian
          //          if (jacobian_scaling_) {
          //            lb->scaleJp_cols(*jacobian_scaling_);
          //          }

          //          // 2. dampen landmarks
          //          lb->setLandmarkDamping(lambda_);

          //          // 3. compute block diagonal preconditioner
          //          (SCHUR_JACOBI) if (precond_block_diagonal_accum_) {
          //            lb->addQ2JpTQ2Jp_blockdiag(*precond_block_diagonal_accum_);
          //          }

          //          // 4. compute rhs of reduced camera normal equations
          //          lb->addQ2JpTQ2r(bref_);
        }
      }

      Reductor(Reductor& a, tbb::split /*unused*/)
          : num_rows(a.num_rows),
            lambda(a.lambda),
            jacobian_scaling(a.jacobian_scaling),
            landmark_blocks(a.landmark_blocks) {
        if (a.precond_block_diagonal_accum) {
          precond_block_diagonal_accum.reset(
              new BlockDiagonalAccumulator<Scalar>());
        }
        bref.setZero(num_rows);
      }

      inline void join(Reductor& b) {
        if (precond_block_diagonal_accum) {
          precond_block_diagonal_accum->join(*b.precond_block_diagonal_accum);
        }
        bref += b.bref;
      }

      size_t num_rows;
      Scalar lambda;
      const VecX* jacobian_scaling;
      std::vector<LandmarkBlockPtr>& landmark_blocks;

      std::unique_ptr<BlockDiagonalAccumulator<Scalar>>
          precond_block_diagonal_accum;
      VecX bref;
    };

    const bool compute_precond_block_diagonal = precond_block_diagonal;

    Reductor r(num_cameras_ * POSE_SIZE, lambda, jacobian_scaling,
               compute_precond_block_diagonal, landmark_blocks_);

    tbb::blocked_range<size_t> range(0, landmark_block_idx_.size());
    tbb::parallel_reduce(range, r);

    // add pose damping to preconditioners
    if (has_pose_damping()) {
      if (compute_precond_block_diagonal) {
        r.precond_block_diagonal_accum->add_diag(
            num_cameras_, POSE_SIZE,
            VecX::Constant(num_cameras_ * POSE_SIZE, pose_damping_diagonal_));
      }
    }

    // TODO: try these kinds of optimizations with std::move here and elsewhere:
    // precond_block_diagonal =
    //   std::move(r.precond_block_diagonal_accum->block_diagonal);
    // bref = std::move(r.bref);

    // move compute results to output variables
    if (compute_precond_block_diagonal) {
      *precond_block_diagonal =
          std::move(r.precond_block_diagonal_accum->block_diagonal);
    }
    bref = std::move(r.bref);
  }

  void print_block(const std::string& filename, size_t block_idx) {
    landmark_blocks_[block_idx].printStorage(filename);
  }

  size_t num_cols() const override { return this->num_cols_reduced(); }

  VecX right_multiply(const VecX& x) const override {
    return this->get_Q2TJp_T_Q2TJp_mult_x(x);
  }

 protected:
  Options options_;

  std::vector<LandmarkBlockPtr> landmark_blocks_;
  std::vector<size_t> landmark_block_idx_;
  BalProblem<Scalar>& bal_problem_;

  mutable std::vector<std::mutex> pose_mutex_;

  Scalar pose_damping_diagonal_ = 0;
  Scalar pose_damping_diagonal_sqrt_ = 0;

  size_t num_cameras_ = 0;
  size_t num_rows_Q2Tr_ = 0;
};

}  // namespace rootba
