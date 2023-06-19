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

#include "rootba/solver/linearizor_base.hpp"

namespace rootba {

template <typename Scalar, int POSE_SIZE>
class LinearizationSC;

template <class Scalar_>
class LinearizorSC : public LinearizorBase<Scalar_> {
 public:
  using Scalar = Scalar_;
  using Base = LinearizorBase<Scalar>;
  constexpr static int POSE_SIZE = 9;

  using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

 public:  // public interface
  LinearizorSC(BalProblem<Scalar>& bal_problem, const SolverOptions& options,
               SolverSummary* summary = nullptr);

  ~LinearizorSC() override;

  void linearize() override;

  VecX solve(Scalar lambda) override;

  Scalar apply(VecX&& inc) override;

 private:
  using Base::bal_problem_;
  using Base::it_summary_;
  using Base::options_;
  using Base::summary_;

  std::unique_ptr<LinearizationSC<Scalar, POSE_SIZE>> lsc_;

  // set during linearization, used in solve
  VecX pose_jacobian_scaling_;

  // indicates if we call solve the first time since the last linearization
  // (first inner iteration for LM-backtracking); true after `linearize`, false
  // after `solve`;
  bool new_linearization_point_ = false;
};

}  // namespace rootba
