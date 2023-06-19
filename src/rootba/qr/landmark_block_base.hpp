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

#include "rootba/qr/landmark_block.hpp"

namespace rootba {

template <class T, typename Scalar, int POSE_SIZE>
class LandmarkBlockBase : public LandmarkBlock<Scalar> {
 public:
  using Options = typename LandmarkBlock<Scalar>::Options;
  using State = typename LandmarkBlock<Scalar>::State;

  bool is_numerical_failure() const override;

  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
  using Vec4 = Eigen::Matrix<Scalar, 4, 1>;

  using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  using Mat36 = Eigen::Matrix<Scalar, 3, 6>;

  using MatX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
  using RowMatX =
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  using Landmark = typename BalProblem<Scalar>::Landmark;
  using Camera = typename BalProblem<Scalar>::Camera;
  using Landmarks = typename BalProblem<Scalar>::Landmarks;
  using Cameras = typename BalProblem<Scalar>::Cameras;

  void allocate_landmark(Landmark& lm, const Options& options) override;

  // may set state to NumericalFailure --> linearization at this state is
  // unusable. Numeric check is only performed for residuals that were
  // considered to be used (valid), which depends on
  // use_valid_projections_only setting.
  void linearize_landmark(const Cameras& cameras) override;

  void perform_qr() override;

  // Sets damping and maintains upper triangular matrix for landmarks.
  void set_landmark_damping(Scalar lambda) override;

  // lambda < 0 means computing exact model cost change
  void back_substitute(const VecX& pose_inc, Scalar& l_diff) override;

  size_t num_reduced_cams() const override;

  size_t num_Q2T_rows() const override;

  Vec3 get_Q1Tr() const;

  void add_triplets_Q2TJp(
      size_t row_offset,
      std::vector<Eigen::Triplet<Scalar>>& triplets) const override;

  Vec3 get_Q1TJp_postmult_x(const VecX& x_pose) const override;

  VecX get_Q2TJp_postmult_x(const VecX& x_pose) const override;

  void add_Q2TJp_premult_x(VecX& res, const VecX& x_r) const override;

  void add_Q2TJp_T_Q2TJp_mult_x(
      VecX& res, const VecX& x_pose,
      std::vector<std::mutex>* pose_mutex = nullptr) const override;

  void add_Q2TJp_T_Q2Tr(VecX& res) const override;

  void add_Q2TJp_diag2(VecX& res) const override;

  void add_Jp_diag2(VecX& res) const override;

  void add_Q2TJp_T_Q2TJp_blockdiag(
      BlockDiagonalAccumulator<Scalar>& accu,
      std::vector<std::mutex>* pose_mutex = nullptr) const override;

  void add_Jp_T_Jp_blockdiag(
      BlockDiagonalAccumulator<Scalar>& accu) const override;

  void scale_Jl_cols() override;

  void scale_Jp_cols(const VecX& jacobian_scaling) override;

  bool has_landmark_damping() const;

  void print_storage(const std::string& filename) const override;

  void stage2(Scalar lambda, const VecX* jacobian_scaling,
              BlockDiagonalAccumulator<Scalar>* precond_block_diagonal,
              VecX& bref) override;

  // debugging / testing
  RowMatX get_Q1TJp(size_t num_cams) const;

  RowMatX get_Q2TJp(size_t num_cams) const;

  State get_state() const override;

  const std::vector<size_t>& get_pose_idx() const override;

 protected:
  void perform_qr_givens();

  void perform_qr_householder();

  Vec3 Jl_col_scale_;
  std::vector<Eigen::JacobiRotation<Scalar>> damping_rotations_;

  Options options_;

  State state_ = State::UNINITIALIZED;

  Landmark* lm_ptr_ = nullptr;
};

}  // namespace rootba
