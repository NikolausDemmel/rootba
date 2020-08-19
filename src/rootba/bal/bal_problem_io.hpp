/**
BSD 3-Clause License

This file is part of the RootBA project.
https://github.com/NikolausDemmel/rootba

Copyright (c) 2021, Nikolaus Demmel.
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

#include <basalt/serialization/eigen_io.h>
#include <basalt/serialization/headers_serialization.h>
#include <cereal/types/map.hpp>
#include <cereal/types/vector.hpp>

#include "rootba/bal/bal_problem.hpp"
#include "rootba/util/serialization.hpp"

namespace rootba {

static const auto BAL_PROBLEM_FILE_INFO = FileInfo{"rootba::BalProblem", "1.0"};

}

namespace cereal {

// TODO: why does it not work when we add a Scalar template argument for
// serializers?

template <class Archive>
void serialize(Archive& ar,
               typename rootba::BalProblem<double>::Observation& obj) {
  ar(CEREAL_NVP_("pos", obj.pos));
}

template <class Archive>
void serialize(Archive& ar, typename rootba::BalProblem<double>::Camera& obj) {
  ar(CEREAL_NVP_("T_c_w", obj.T_c_w),
     CEREAL_NVP_("intrinsics", obj.intrinsics));
  // NOTE: we don't serialize the 'backup' variables
}

template <class Archive>
void serialize(Archive& ar,
               typename rootba::BalProblem<double>::Landmark& obj) {
  ar(CEREAL_NVP_("p_w", obj.p_w), CEREAL_NVP_("obs", obj.obs));
  // NOTE: we don't serialize the 'backup' variables
}

template <class Archive, class Scalar>
void serialize(Archive& ar, rootba::BalProblem<Scalar>& obj) {
  ar(make_nvp("cameras", obj.cameras()),
     make_nvp("landmarks", obj.landmarks()));
}

}  // namespace cereal
