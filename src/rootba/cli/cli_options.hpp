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

#include <clipp/clipp.h>
#include <nlohmann/json.hpp>

#include "rootba/options/options_interface.hpp"

namespace rootba {

inline std::string to_cli_name(std::string name) {
  std::replace(name.begin(), name.end(), '_', '-');
  return name;
}

// std::string to_upper(std::string str) {
//  for (auto& c : str) c = toupper((unsigned char)c);
//  return str;
//}

template <class T>
std::string cli_placeholder() {
  if constexpr (std::is_same_v<T, int>) {
    return "INT";
  } else if constexpr (std::is_same_v<T, double>) {
    return "FLOAT";
  } else if constexpr (std::is_same_v<T, std::string>) {
    return "STR";
  }
}

class CliArgumentsOptionsVisitor : public ConstOptionsVisitor {
  using json = nlohmann::json;

 public:
  CliArgumentsOptionsVisitor(json& output_node, const std::string& cli_prefix);

  void operator()(const char* name, OptionsVariableConstRef var,
                  const OptionsMeta& meta) override;

  void operator()(const char* name, const OptionsInterface& child) override;

  inline clipp::group get_cli_group() const { return cli_; }

 private:
  std::vector<json::json_pointer> json_prefix_stack_;
  std::vector<std::string> cli_prefix_stack_;
  json& output_node_;
  clipp::group cli_;
};

// Returns cli group for given options (recursive). When parsing cli arguments,
// the actually passed arguments are then stored in the json object
// parse_output. You can then pass this json object to options._load() to set
// the parsed values.
clipp::group cli_options(nlohmann::json& parse_output,
                         const OptionsInterface& options,
                         const std::string& prefix);

}  // namespace rootba
