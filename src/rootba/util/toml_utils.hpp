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

#include <glog/logging.h>
#include <toml11/toml.hpp>

#include "rootba/util/format.hpp"
#include "rootba/util/json_utils.hpp"
#include "rootba/util/template_utils.hpp"

namespace rootba {

// toml type that preserves the insertion order of tables
using ordered_toml = toml::basic_value<toml::discard_comments,
                                       nlohmann::ordered_map, std::vector>;

// TODO: restrict to json and toml types
template <class J, class T>
auto toml_to_json(const T& toml_obj, J& json_obj)
    -> std::enable_if_t<is_basic_json_v<J>> {
  // clear to null object
  json_obj = J();

  // convert known types
  toml::visit(
      mp::overload{
          [&](const typename T::boolean_type& val) { json_obj = val; },
          [&](const typename T::integer_type& val) { json_obj = val; },
          [&](const typename T::floating_type& val) { json_obj = val; },
          [&](const typename T::string_type& val) { json_obj = val.str; },
          [&](const typename T::table_type& val) {
            json_obj = J::object();
            for (const auto& [k, v] : val) {
              toml_to_json(v, json_obj[k]);
            }
          },
          [&](const typename T::array_type& val) {
            json_obj = J::array();
            for (const auto& v : val) {
              json_obj.emplace_back();
              toml_to_json(v, json_obj.back());
            }
          },
          [&](const auto& val) {
            ROOTBA_UNUSED(val);
            json_obj = J();
            LOG(ERROR)
                << "Conversion to json not yet implemented for toml type {}."_format(
                       toml::stringize(toml_obj.type()));
          }},
      toml_obj);
}

// TODO: restrict to json and toml types
template <class J, class T>
auto json_to_toml(const J& json_obj, T& toml_obj)
    -> std::enable_if_t<is_basic_json_v<J>> {
  switch (json_obj.type()) {
    case J::value_t::null:
    case J::value_t::discarded:
      toml_obj = toml::value();
      break;
    case J::value_t::boolean:
      toml_obj = json_obj.template get<bool>();
      break;
    case J::value_t::string:
      toml_obj = json_obj.template get<typename J::string_t>();
      break;
    case J::value_t::number_integer:
      toml_obj = json_obj.template get<typename J::number_integer_t>();
      break;
    case J::value_t::number_unsigned:
      toml_obj = json_obj.template get<typename J::number_unsigned_t>();
      break;
    case J::value_t::number_float:
      toml_obj = json_obj.template get<typename J::number_float_t>();
      break;
    case J::value_t::object: {
      typename T::table_type tmp;
      for (const auto& [k, v] : json_obj.items()) {
        json_to_toml(v, tmp[k]);
      }
      toml_obj = std::move(tmp);
      break;
    }
    case J::value_t::array: {
      typename T::array_type tmp;
      for (const auto& el : json_obj) {
        T tmp_el;
        json_to_toml(el, tmp_el);
        tmp.emplace_back(std::move(tmp_el));
      }
      toml_obj = std::move(tmp);
      break;
    }
    default:
      LOG(FATAL) << "unreachable";
  }
}

}  // namespace rootba
