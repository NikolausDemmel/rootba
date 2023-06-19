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
#include "rootba/solver/linearizor.hpp"

#include <magic_enum/magic_enum.hpp>

#include "rootba/solver/linearizor_power_sc.hpp"
#include "rootba/solver/linearizor_qr.hpp"
#include "rootba/solver/linearizor_sc.hpp"
#include "rootba/util/format.hpp"

namespace rootba {

template <typename Scalar_>
std::unique_ptr<Linearizor<Scalar_>> Linearizor<Scalar_>::create(
    BalProblem<Scalar>& bal_problem, const SolverOptions& options,
    SolverSummary* summary) {
  switch (options.solver_type) {
    case SolverOptions::SolverType::SQUARE_ROOT:
      return std::make_unique<LinearizorQR<Scalar>>(bal_problem, options,
                                                    summary);
    case SolverOptions::SolverType::SCHUR_COMPLEMENT:
      return std::make_unique<LinearizorSC<Scalar>>(bal_problem, options,
                                                    summary);
    case SolverOptions::SolverType::POWER_SCHUR_COMPLEMENT:
      return std::make_unique<LinearizorPowerSC<Scalar>>(bal_problem, options,
                                                         summary);
    default:
      LOG(FATAL) << "Invalid LinearizorType {}"_format(
          magic_enum::enum_name(options.solver_type));
  }
}

#ifdef ROOTBA_INSTANTIATIONS_FLOAT
template class Linearizor<float>;
#endif

#ifdef ROOTBA_INSTANTIATIONS_DOUBLE
template class Linearizor<double>;
#endif

}  // namespace rootba
