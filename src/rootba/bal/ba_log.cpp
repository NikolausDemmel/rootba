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

#include "rootba/bal/ba_log.hpp"

#include <fstream>
#include <iomanip>
#include <type_traits>

#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include "rootba/bal/ba_log_options.hpp"
#include "rootba/util/format.hpp"
#include "rootba/util/template_utils.hpp"
#include "rootba/util/visit_struct_utils.hpp"

namespace rootba {

void BaLog::clear() {
  decltype(iterations)().swap(iterations);  // clear memory
  static_data = Static();
}

bool BaLog::save_json(const BaLogOptions& options) const {
  return save_json(options.log_path, options.save_log_flags);
}

bool BaLog::save_json(const std::string& path, const SaveLogFlags flags) const {
  // short circute of no filetype to save is selected
  if (flags.empty()) {
    return true;
  }

  using json = nlohmann::json;
  json result;

  // construct a flat list of values for easier import in matlab
  for (const auto& iter : iterations) {
    // direct members
    visit_struct::for_each(
        iter,
        mp::overload{[&](const char* name, const bool& value) {
                       result[name].push_back(value);
                     },
                     [&](const char* name, const int& value) {
                       result[name].push_back(value);
                     },
                     [&](const char* name, const uint64_t& value) {
                       result[name].push_back(value);
                     },
                     [&](const char* name, const double& value) {
                       result[name].push_back(value);
                     },
                     [&](const char* name, const std::string& value) {
                       result[name].push_back(value);
                     },
                     [&](const char* name, const std::vector<double>& value) {
                       result[name].push_back(json::array());
                       auto& arr = result[name].back();
                       for (const double x : value) {
                         arr.push_back(x);
                       }
                     }});
  }

  // helper
  auto set_warn_if_exists = [&](const std::string& name, auto value) {
    if (result.count(name)) {
      LOG(WARNING) << "Per-iteration log entry '" << name
                   << "' is being overwritten";
    }
    result[name] = std::move(value);
  };

  // Set type for easier parsing
  set_warn_if_exists("_type", "rootba");

  // static info (doesn't change over iterations)
  set_warn_if_exists("_static", visitable_to_json(static_data));

  // save json
  if (flags & SaveLogFlag::JSON) {
    std::ofstream ofs(path);

    if (!ofs.is_open()) {
      LOG(ERROR) << "Could not save BA log to {}."_format(path);
      return false;
    }

    ofs << std::setw(4) << result;  //!< pretty printing
    // ofs << result;  //!< no pretty printing

    LOG(INFO) << "Saved log for {} iterations to {}."_format(iterations.size(),
                                                             path);
  }

  // save ubjson
  if (flags & SaveLogFlag::UBJSON) {
    std::string ubjson_path =
        path.substr(0, path.find_last_of('.')) + ".ubjson";
    std::ofstream ofs(ubjson_path, std::ios_base::binary);

    if (!ofs.is_open()) {
      LOG(ERROR) << "Could not save BA log to {}."_format(ubjson_path);
      return false;
    }

    json::to_ubjson(result, ofs);

    LOG(INFO) << "Saved log for {} iterations to {}."_format(iterations.size(),
                                                             ubjson_path);
  }

  return true;
}

}  // namespace rootba
