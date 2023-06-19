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

#include <Eigen/Dense>
#include <basalt/utils/sophus_utils.hpp>
#include <glog/logging.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

#include "rootba/bal/bal_problem.hpp"
#include "rootba/cg/conjugate_gradient.hpp"
#include "rootba/sc/landmark_block.hpp"
#include "rootba/sc/linearization_sc.hpp"
#include "rootba/util/assert.hpp"

namespace rootba {

template <typename Scalar_, int POSE_SIZE_>
class LinearizationPowerSC : private LinearizationSC<Scalar_, POSE_SIZE_> {
 public:
  using Scalar = Scalar_;
  static constexpr int POSE_SIZE = POSE_SIZE_;
  using Base = LinearizationSC<Scalar_, POSE_SIZE_>;

  using Vec2 = Eigen::Matrix<Scalar_, 2, 1>;
  using Vec4 = Eigen::Matrix<Scalar_, 4, 1>;
  using VecX = Eigen::Matrix<Scalar_, Eigen::Dynamic, 1>;
  using MatX = Eigen::Matrix<Scalar_, Eigen::Dynamic, Eigen::Dynamic>;
  using Mat36 = Eigen::Matrix<Scalar_, 3, 6>;
  using RowMatX =
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  using PerSolveOptions =
      typename ConjugateGradientsSolver<Scalar>::PerSolveOptions;
  using SparseMat = Eigen::SparseMatrix<Scalar, Eigen::RowMajor>;

  using Options = typename LandmarkBlockSC<Scalar, POSE_SIZE>::Options;

  struct Summary {
    enum TerminationType {
      LINEAR_SOLVER_NO_CONVERGENCE,
      LINEAR_SOLVER_SUCCESS,
      LINEAR_SOLVER_FAILURE
    };

    TerminationType termination_type;
    std::string message;
    int power_order = 0;
  };

  LinearizationPowerSC(BalProblem<Scalar>& bal_problem, const Options& options)
      : Base(bal_problem, options, false),
        Hpp_inv_(POSE_SIZE * num_cameras_, POSE_SIZE) {
    ROOTBA_ASSERT(pose_mutex_.size() == num_cameras_);
  }

  // Prepare the components required for solving:
  // b_p, Hpp_inv, and Hll_inv (cached inside landmark block)
  void prepare_Hb(VecX& b_p) {
    Hpp_inv_.setZero(POSE_SIZE * num_cameras_, POSE_SIZE);
    b_p.setZero(num_cameras_ * POSE_SIZE);

    {
      auto body = [&](const tbb::blocked_range<size_t>& range) {
        for (size_t r = range.begin(); r != range.end(); ++r) {
          const auto& lb = landmark_blocks_[r];
          lb.stage(b_p, &Hpp_inv_, nullptr, &pose_mutex_);
        }
      };

      tbb::blocked_range<size_t> range(0, landmark_blocks_.size());
      tbb::parallel_for(range, body);
    }

    {
      auto body = [&](const tbb::blocked_range<size_t>& range) {
        for (size_t r = range.begin(); r != range.end(); ++r) {
          auto Hpp_inv =
              Hpp_inv_.template block<POSE_SIZE, POSE_SIZE>(POSE_SIZE * r, 0);
          Hpp_inv.diagonal().array() += pose_damping_diagonal_;
          Hpp_inv =
              Hpp_inv.template selfadjointView<Eigen::Upper>().llt().solve(
                  MatX::Identity(POSE_SIZE, POSE_SIZE));
        }
      };

      tbb::blocked_range<size_t> range(0, num_cameras_);
      tbb::parallel_for(range, body);
    }
  }

  // Solve RCS (Eq. 23)
  Summary solve(const VecX& b_p, VecX& accum, PerSolveOptions& pso) const {
    ROOTBA_ASSERT(static_cast<size_t>(b_p.size()) == POSE_SIZE * num_cameras_);

    Summary summary;

    accum = right_mul_Hpp_inv(-b_p);
    VecX tmp = accum;

    for (size_t i = 1; i <= options_.power_order; ++i) {
      tmp = right_mul_Hpp_inv(right_mul_e0(tmp));
      accum += tmp;

      // Convergence check
      if (pso.q_tolerance > 0) {
        const Scalar zeta = i * tmp.norm() / accum.norm();
        if (zeta < pso.q_tolerance) {
          summary.termination_type = Summary::LINEAR_SOLVER_SUCCESS;
          summary.power_order = i;
          std::stringstream ss;
          ss << "Iteration: " << summary.power_order
             << " Convergence. zeta = " << zeta << " < " << pso.q_tolerance;
          summary.message = ss.str();
          return summary;
        }
      }
    }

    summary.termination_type = Summary::LINEAR_SOLVER_NO_CONVERGENCE;
    summary.power_order = options_.power_order;
    summary.message = "Maximum number of iterations reached.";
    return summary;
  }

  // Inverse Schur complement (Eq. 17)
  MatX get_inverted_matrix() const {
    SparseMat Jp = get_sparse_Jp();
    SparseMat Jl = get_sparse_Jl();

    SparseMat Hpp_inv = get_sparse_Hpp_inv();
    SparseMat Hll_inv = get_sparse_Hll_inv();

    const size_t rows = num_cameras_ * POSE_SIZE;
    MatX mat = MatX::Identity(rows, rows) -
               Hpp_inv * Jp.transpose() * Jl * Hll_inv * Jl.transpose() * Jp;

    return mat.inverse() * Hpp_inv;
  }

  // Inverse Schur complement constructed via power series (Eq. 22)
  MatX get_inverted_matrix(size_t order) const {
    SparseMat Jp = get_sparse_Jp();
    SparseMat Jl = get_sparse_Jl();

    SparseMat Hpp_inv = get_sparse_Hpp_inv();
    SparseMat Hll_inv = get_sparse_Hll_inv();

    MatX accm = Hpp_inv;
    MatX tmp = accm;
    for (size_t i = 1; i <= order; ++i) {
      tmp = Hpp_inv * Jp.transpose() * Jl * Hll_inv * Jl.transpose() * Jp * tmp;
      accm += tmp;
    }

    // sum_i^{order} (Hpp^-1 * Hpl * Hll^-1 * Hlp)^i * Hpp^-1
    return accm;
  }

  void print_block(const std::string& filename, size_t block_idx) {
    landmark_blocks_[block_idx].printStorage(filename);
  }

  // For debugging only
  inline auto get_Hpp_inv(const size_t cam_idx) const {
    return Hpp_inv_.template block<POSE_SIZE, POSE_SIZE>(POSE_SIZE * cam_idx,
                                                         0);
  }

  // For debugging only
  VecX right_multiply(const VecX& x) const {
    const auto jacobi = Base::get_jacobi();

    VecX res(POSE_SIZE * num_cameras_);
    for (size_t i = 0; i < num_cameras_; ++i) {
      const auto u = jacobi.block_diagonal.at(std::make_pair(i, i));
      const auto v = x.template segment<POSE_SIZE>(POSE_SIZE * i);
      res.template segment<POSE_SIZE>(POSE_SIZE * i) = u * v;
    }

    res -= right_mul_e0(x);
    return res;
  }

  // make selected base class methods public
  using Base::back_substitute;
  using Base::compute_Jp_scale_and_scale_Jp_cols;
  using Base::get_Jp_diag2;
  using Base::linearize_problem;
  using Base::num_cols_reduced;
  using Base::scale_Jl_cols;
  using Base::scale_Jp_cols;
  using Base::set_landmark_damping;
  using Base::set_pose_damping;

  inline VecX right_mul_Hpp_inv(const VecX& x) const {
    ROOTBA_ASSERT(static_cast<size_t>(x.size()) == num_cameras_ * POSE_SIZE);

    VecX res(num_cameras_ * POSE_SIZE);

    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        const auto u =
            Hpp_inv_.template block<POSE_SIZE, POSE_SIZE>(POSE_SIZE * r, 0);
        const auto v = x.template segment<POSE_SIZE>(POSE_SIZE * r);
        res.template segment<POSE_SIZE>(POSE_SIZE * r) = u * v;
      }
    };

    tbb::blocked_range<size_t> range(0, num_cameras_);
    tbb::parallel_for(range, body);

    return res;
  }

  inline VecX right_mul_e0(const VecX& x) const {
    ROOTBA_ASSERT(static_cast<size_t>(x.size()) == num_cameras_ * POSE_SIZE);

    VecX res = VecX::Zero(num_cameras_ * POSE_SIZE);

    auto body = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        const auto& lb = landmark_blocks_.at(r);

        VecX Jp_x(2 * lb.num_poses());
        lb.add_Jp_x(Jp_x, x);

        const auto Jl = lb.get_Jl();
        const VecX tmp = Jl * (lb.get_Hll_inv() * (Jl.transpose() * Jp_x));

        lb.add_JpT_x(res, tmp, &pose_mutex_);
      }
    };

    tbb::blocked_range<size_t> range(0, landmark_blocks_.size());
    tbb::parallel_for(range, body);

    return res;
  }

 protected:
  // For debugging only
  SparseMat get_sparse_Jp() const {
    std::vector<size_t> lm_obs_indices;
    size_t num_obs = 0;
    lm_obs_indices.reserve(landmark_blocks_.size());
    for (const auto& lm_block : landmark_blocks_) {
      lm_obs_indices.push_back(num_obs);
      num_obs += lm_block.num_poses();
    }

    std::vector<Eigen::Triplet<Scalar>> triplets;
    triplets.reserve(num_obs * 2 * POSE_SIZE);

    for (size_t lm_idx = 0; lm_idx < landmark_blocks_.size(); ++lm_idx) {
      const auto& lb = landmark_blocks_[lm_idx];
      const size_t row_idx = lm_obs_indices[lm_idx] * 2;

      const auto& pose_indices = lb.get_pose_idx();
      for (size_t j = 0; j < pose_indices.size(); ++j) {
        const auto block = lb.get_Jpi(j);
        const size_t col_idx = pose_indices[j];

        const size_t row_offset = row_idx + j * 2;
        const size_t col_offset = col_idx * POSE_SIZE;

        for (Eigen::Index row = 0; row < 2; ++row) {
          for (Eigen::Index col = 0; col < POSE_SIZE; ++col) {
            triplets.emplace_back(row + row_offset, col + col_offset,
                                  block(row, col));
          }
        }
      }
    }

    // build sparse matrix
    SparseMat res(num_obs * 2, num_cameras_ * POSE_SIZE);
    if (!triplets.empty()) {
      res.setFromTriplets(triplets.begin(), triplets.end());
    }

    return res;
  }

  SparseMat get_sparse_Jl() const {
    std::vector<size_t> lm_obs_indices;
    size_t num_obs = 0;
    lm_obs_indices.reserve(landmark_blocks_.size());
    for (const auto& lm_block : landmark_blocks_) {
      lm_obs_indices.push_back(num_obs);
      num_obs += lm_block.num_poses();
    }

    std::vector<Eigen::Triplet<Scalar>> triplets;
    triplets.reserve(num_obs * 2 * 3);

    for (size_t lm_idx = 0; lm_idx < landmark_blocks_.size(); ++lm_idx) {
      const auto& lb = landmark_blocks_[lm_idx];

      const size_t row_idx = lm_obs_indices[lm_idx] * 2;
      const size_t col_offset = lm_idx * 3;

      const size_t num_lm_obs = lb.get_pose_idx().size();
      for (size_t j = 0; j < num_lm_obs; ++j) {
        const auto block = lb.get_Jli(j);
        const size_t row_offset = row_idx + j * 2;

        for (Eigen::Index row = 0; row < 2; ++row) {
          for (Eigen::Index col = 0; col < 3; ++col) {
            triplets.emplace_back(row + row_offset, col + col_offset,
                                  block(row, col));
          }
        }
      }
    }

    // build sparse matrix
    SparseMat res(num_obs * 2, landmark_blocks_.size() * 3);
    if (!triplets.empty()) {
      res.setFromTriplets(triplets.begin(), triplets.end());
    }

    return res;
  }

  SparseMat get_sparse_Hll_inv() const {
    BlockSparseMatrix<Scalar, 3> Hll_inv(3 * landmark_blocks_.size(),
                                         3 * landmark_blocks_.size());
    for (size_t i = 0; i < landmark_blocks_.size(); ++i) {
      const auto& lb = landmark_blocks_[i];
      Eigen::Matrix<Scalar, 3, 3> m = lb.get_Hll_inv();
      Hll_inv.add(i, i, std::move(m));
    }
    return Hll_inv.get_sparse_matrix();
  }

  SparseMat get_sparse_Hpp_inv() const {
    BlockDiagonalAccumulator<Scalar> Hpp;
    {
      auto body = [&](const tbb::blocked_range<size_t>& range) {
        for (size_t r = range.begin(); r != range.end(); ++r) {
          const auto& lb = landmark_blocks_[r];
          lb.add_Hpp(Hpp, pose_mutex_);
        }
      };
      tbb::blocked_range<size_t> range(0, landmark_blocks_.size());
      tbb::parallel_for(range, body);

      if (pose_damping_diagonal_ > 0) {
        Hpp.add_diag(
            num_cameras_, POSE_SIZE,
            VecX::Constant(num_cameras_ * POSE_SIZE, pose_damping_diagonal_));
      }
    }

    BlockSparseMatrix<Scalar, POSE_SIZE> Hpp_inv(num_cameras_ * POSE_SIZE,
                                                 num_cameras_ * POSE_SIZE);
    {
      auto body = [&](const typename IndexedBlocks<Scalar>::range_type& range) {
        for (const auto& [idx, block] : range) {
          Eigen::Matrix<Scalar, POSE_SIZE, POSE_SIZE> inv_block =
              block.inverse();
          Hpp_inv.add(idx.first, idx.second, std::move(inv_block));
        }
      };
      tbb::parallel_for(Hpp.block_diagonal.range(), body);
    }

    return Hpp_inv.get_sparse_matrix();
  }

  using Base::landmark_blocks_;
  using Base::num_cameras_;
  using Base::options_;
  using Base::pose_damping_diagonal_;
  using Base::pose_mutex_;

  RowMatX Hpp_inv_;
};

}  // namespace rootba
