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

#include "rootba/cli/cli_options.hpp"

#include "rootba/util/format.hpp"
#include "rootba/util/pprint_utils.hpp"

namespace rootba {

CliArgumentsOptionsVisitor::CliArgumentsOptionsVisitor(
    json& output_node, const std::string& cli_prefix)
    : output_node_(output_node) {
  json_prefix_stack_.emplace_back();
  cli_prefix_stack_.push_back(cli_prefix.empty() ? "" : cli_prefix + "-");
}

void CliArgumentsOptionsVisitor::operator()(const char* name,
                                            const OptionsVariableConstRef var,
                                            const OptionsMeta& meta) {
  // skip flags disabled for cli
  if (meta.flags & OptionsMetaFlag::HIDE_CLI) {
    return;
  }

  using namespace clipp;
  const auto& cli_prefix = cli_prefix_stack_.back();
  auto cli_arg = "--{}{}"_format(cli_prefix, to_cli_name(name));
  auto json_ptr = json_prefix_stack_.back() / name;
  std::string help = meta.help.empty() ? std::string("") : meta.help;
  auto& output = output_node_;

  std::visit(
      [&](auto val) {
        using WrapperT = decltype(val);
        if constexpr (mp::is_instance_v<WrapperT, std::reference_wrapper>) {
          using ValueT = std::decay_t<decltype(val.get())>;
          if constexpr (mp::is_instance_v<ValueT, std::vector>) {
            help += "\n";
            using ElemT = typename ValueT::value_type;
            cli_.push_back(option(cli_arg) &
                           values(cli_placeholder<ElemT>())
                                   .call([=, &output](const char* s) {
                                     output[json_ptr].push_back(
                                         clipp::detail::make<ElemT>::from(s));
                                   }) %
                               help);
          } else if constexpr (std::is_same_v<ValueT, bool>) {
            if (help.empty()) {
              help = " ";  // fix doc rendering
            }
            auto cli_no_arg = "--no-{}{}"_format(cli_prefix, to_cli_name(name));
            cli_.push_back((option(cli_arg).call(
                                [=, &output] { output[json_ptr] = true; }) |
                            option(cli_no_arg).call([=, &output] {
                              output[json_ptr] = false;
                            })) %
                           help);
          } else {
            help += "\n";
            cli_.push_back(option(cli_arg) &
                           value(cli_placeholder<ValueT>())
                                   .call([=, &output](const char* s) {
                                     output[json_ptr] =
                                         clipp::detail::make<ValueT>::from(s);
                                   }) %
                               help);
          }
        } else if constexpr (std::is_same_v<WrapperT, EnumConstRefWrapper>) {
          help += "\n";
          // parse as string
          auto values_help = "(possible values: {})"_format(
              pprint_compact(val.get_possible_values()));
          help = help.empty() ? values_help : help + " " + values_help;
          cli_.push_back(option(cli_arg) &
                         value("ENUM").call([=, &output](const std::string& s) {
                           output[json_ptr] = s;
                         }) % help);
        } else if constexpr (std::is_same_v<WrapperT, FlagsConstWrapper>) {
          help += "\n";
          // parse as list of strings
          auto values_help = "(possible flags: {})"_format(
              pprint_compact(val.get_possible_flags()));
          help = help.empty() ? values_help : help + " " + values_help;
          cli_.push_back(
              option(cli_arg) &
              values("FLAG").call([=, &output](const std::string& s) {
                output[json_ptr].push_back(s);
              }) % help);
        } else {
          static_assert(mp::always_false_v<WrapperT>,
                        "non-exhaustive visitor!");
        }
      },
      var);
}

void CliArgumentsOptionsVisitor::operator()(const char* name,
                                            const OptionsInterface& child) {
  json_prefix_stack_.push_back(json_prefix_stack_.back() / name);
  cli_prefix_stack_.push_back("{}{}-"_format(cli_prefix_stack_.back(), name));
  child._accept(*this);
  json_prefix_stack_.pop_back();
  cli_prefix_stack_.pop_back();
}

clipp::group cli_options(nlohmann::json& parse_output,
                         const OptionsInterface& options,
                         const std::string& prefix) {
  CliArgumentsOptionsVisitor visitor(parse_output, prefix);
  options._accept(visitor);
  return visitor.get_cli_group();
}

}  // namespace rootba
