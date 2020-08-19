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

#include "rootba/options/visitable_options.hpp"

namespace rootba {

/// Options relevant for evaluation the BA residuals, e.g., which robust norm to
/// use.
struct BalResidualOptions : public VisitableOptions<BalResidualOptions> {
  WISE_ENUM_CLASS_MEMBER(RobustNorm,
                         NONE,  //!< squared norm (not robust)
                         HUBER  //!< huber norm
  );

  BEGIN_VISITABLES(BalResidualOptions);

  VISITABLE_META(RobustNorm, robust_norm,
                 init(RobustNorm::NONE)
                     .help("which robust norm to use. NONE: squared norm, "
                           "HUBER: Huber norm."));

  // ceres uses value of 1.0 in bundle_adjuster example
  VISITABLE_META(
      double, huber_parameter,
      init(1).range(0, 10).help("huber parameter for robust norm in pixels"));

  END_VISITABLES;
};

}  // namespace rootba
