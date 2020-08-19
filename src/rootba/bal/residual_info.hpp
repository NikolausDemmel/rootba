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

#include <string>

namespace rootba {

// positive values mean cost was reduced, i.e.:
// change = previous_iteration - current_iteration
struct ResidualChangeItem {
  int num_obs = 0;
  double error = 0;
  double residual_sum = 0;
  double error_avg = 0;
  double residual_mean = 0;
};

// residual change items for all and only valid residuals
struct ResidualChangeInfo {
  ResidualChangeItem all;
  ResidualChangeItem valid;
};

// summary of ba-residual and cost
struct ResidualItem {
  int num_obs = 0;
  double error = 0;
  double residual_sum = 0;

  inline double error_avg() const {
    return num_obs > 0 ? error / num_obs : 0.0;
  }

  inline double residual_mean() const {
    return num_obs > 0 ? residual_sum / num_obs : 0.0;
  }

  ResidualChangeItem compared_to(const ResidualItem& previous_iteration) const;

  static ResidualItem join(const ResidualItem& a, const ResidualItem& b);
};

// residual items for all and only valid residuals
struct ResidualInfo {
  ResidualItem all;
  ResidualItem valid;

  // indicate numerical failure during evaluation of residuals that are
  // considered to be used by solver (depends on options)
  bool is_numerically_valid = true;

  ResidualChangeInfo compared_to(const ResidualInfo& previous_iteration) const;

  static ResidualInfo join(const ResidualInfo& a, const ResidualInfo& b);
};

std::string error_summary_oneline(const ResidualInfo& info, bool valid_first);

// accumulator for parallel reduce over all residuals
struct ResidualInfoAccu {
  ResidualInfo info;

  void add(bool numerically_valid, bool projection_valid, double res_norm,
           double weighted_error);

  static ResidualInfoAccu join(const ResidualInfoAccu& a,
                               const ResidualInfoAccu& b);
};

}  // namespace rootba
