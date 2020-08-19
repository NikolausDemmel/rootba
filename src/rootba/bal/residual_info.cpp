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

#include "rootba/bal/residual_info.hpp"

#include "rootba/util/format.hpp"

namespace rootba {

ResidualChangeItem ResidualItem::compared_to(
    const ResidualItem& previous_iteration) const {
  // compute "this - previous_iteration"
  ResidualChangeItem change;
  change.num_obs = previous_iteration.num_obs - num_obs;
  change.error = previous_iteration.error - error;
  change.residual_sum = previous_iteration.residual_sum - residual_sum;
  change.error_avg = previous_iteration.error_avg() - error_avg();
  change.residual_mean = previous_iteration.residual_mean() - residual_mean();
  return change;
}

ResidualChangeInfo ResidualInfo::compared_to(
    const ResidualInfo& previous_iteration) const {
  ResidualChangeInfo change;
  change.all = all.compared_to(previous_iteration.all);
  change.valid = valid.compared_to(previous_iteration.valid);
  return change;
}

ResidualItem ResidualItem::join(const ResidualItem& a, const ResidualItem& b) {
  ResidualItem c;
  c.num_obs = a.num_obs + b.num_obs;
  c.error = a.error + b.error;
  c.residual_sum = a.residual_sum + b.residual_sum;
  return c;
}

ResidualInfo ResidualInfo::join(const ResidualInfo& a, const ResidualInfo& b) {
  ResidualInfo c;
  c.all = ResidualItem::join(a.all, b.all);
  c.valid = ResidualItem::join(a.valid, b.valid);
  c.is_numerically_valid = a.is_numerically_valid && b.is_numerically_valid;
  return c;
}

std::string error_summary_oneline(const ResidualItem& item) {
  return "{:.4e} (mean res: {:.2f}, num: {})"_format(
      item.error, item.residual_mean(), item.num_obs);
}

std::string error_summary_oneline(const ResidualInfo& info, bool valid_first) {
  std::string warning = info.is_numerically_valid ? "" : "!NaN! ";
  if (valid_first) {
    return "{}error valid: {}, error: {}"_format(
        warning, error_summary_oneline(info.valid),
        error_summary_oneline(info.all));
  } else {
    return "{}error: {}, error valid: {}"_format(
        warning, error_summary_oneline(info.all),
        error_summary_oneline(info.valid));
  }
}

void ResidualInfoAccu::add(bool numerically_valid, bool projection_valid,
                           double res_norm, double weighted_error) {
  info.is_numerically_valid &= numerically_valid;

  ++info.all.num_obs;
  info.all.error += weighted_error;
  info.all.residual_sum += res_norm;

  if (projection_valid) {
    ++info.valid.num_obs;
    info.valid.error += weighted_error;
    info.valid.residual_sum += res_norm;
  }
}

ResidualInfoAccu ResidualInfoAccu::join(const ResidualInfoAccu& a,
                                        const ResidualInfoAccu& b) {
  ResidualInfoAccu c;
  c.info = ResidualInfo::join(a.info, b.info);
  return c;
}

}  // namespace rootba
