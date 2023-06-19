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

#include <memory>
#include <mutex>

#include <Eigen/Dense>

#include "rootba/bal/bal_problem.hpp"
#include "rootba/bal/bal_residual_options.hpp"
#include "rootba/cg/block_sparse_matrix.hpp"

namespace rootba {

template <typename Scalar>
class LandmarkBlock {
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

  enum State {
    UNINITIALIZED = 0,
    ALLOCATED,
    NUMERICAL_FAILURE,
    LINEARIZED,
    MARGINALIZED
  };

  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
  using Vec4 = Eigen::Matrix<Scalar, 4, 1>;

  using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  using Mat36 = Eigen::Matrix<Scalar, 3, 6>;

  using MatX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
  using RowMatX =
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  using RowMat3 = Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor>;

  using Landmark = typename BalProblem<Scalar>::Landmark;
  using Camera = typename BalProblem<Scalar>::Camera;
  using Landmarks = typename BalProblem<Scalar>::Landmarks;
  using Cameras = typename BalProblem<Scalar>::Cameras;

  virtual ~LandmarkBlock() = default;

  virtual bool is_numerical_failure() const = 0;
  virtual void allocate_landmark(Landmark& lm, const Options& options) = 0;

  // may set state to NumericalFailure --> linearization at this state is
  // unusable. Numeric check is only performed for residuals that were
  // considered to be used (valid), which depends on
  // use_valid_projections_only setting.
  virtual void linearize_landmark(const Cameras& cameras) = 0;
  virtual void perform_qr() = 0;

  // Sets damping and maintains upper triangular matrix for landmarks.
  virtual void set_landmark_damping(Scalar lambda) = 0;

  // lambda < 0 means computing exact model cost change
  virtual void back_substitute(const VecX& pose_inc, Scalar& l_diff) = 0;
  virtual void add_Q2TJp_T_Q2TJp_mult_x(
      VecX& res, const VecX& x_pose,
      std::vector<std::mutex>* pose_mutex = nullptr) const = 0;

  virtual Vec3 get_Q1TJp_postmult_x(const VecX& x_pose) const = 0;

  virtual VecX get_Q2TJp_postmult_x(const VecX& x_pose) const = 0;

  virtual void add_Q2TJp_premult_x(VecX& res, const VecX& x_r) const = 0;

  virtual void add_Q2TJp_T_Q2Tr(VecX& res) const = 0;

  virtual void add_Q2TJp_diag2(VecX& res) const = 0;
  virtual void add_Jp_diag2(VecX& res) const = 0;

  virtual void add_Q2TJp_T_Q2TJp_blockdiag(
      BlockDiagonalAccumulator<Scalar>& accu,
      std::vector<std::mutex>* pose_mutex = nullptr) const = 0;
  virtual void add_Jp_T_Jp_blockdiag(
      BlockDiagonalAccumulator<Scalar>& accu) const = 0;

  virtual void add_triplets_Q2TJp(
      size_t row_offset,
      std::vector<Eigen::Triplet<Scalar>>& triplets) const = 0;

  virtual void scale_Jl_cols() = 0;
  virtual void scale_Jp_cols(const VecX& jacobian_scaling) = 0;
  virtual void print_storage(const std::string& filename) const = 0;
  virtual State get_state() const = 0;

  virtual size_t num_reduced_cams() const = 0;
  virtual size_t num_Q2T_rows() const = 0;

  virtual void stage2(Scalar lambda, const VecX* jacobian_scaling,
                      BlockDiagonalAccumulator<Scalar>* precond_block_diagonal,
                      VecX& bref) = 0;

  virtual const std::vector<size_t>& get_pose_idx() const = 0;
};

template <typename Scalar, int POSE_SIZE>
class LandmarkBlockFactory {
 public:
  static std::unique_ptr<LandmarkBlock<Scalar>> get_landmark_block(
      size_t obs_size);
};

}  // namespace rootba
