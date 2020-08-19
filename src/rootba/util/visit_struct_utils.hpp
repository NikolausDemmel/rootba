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

#include <nlohmann/json.hpp>
#include <visit_struct/visit_struct.hpp>
#include <wise_enum/wise_enum.h>

#include "rootba/util/container_traits.hpp"
#include "rootba/util/template_utils.hpp"

namespace rootba {

// general equality with visit_struct
struct eq_visitor {  // NOLINT
  bool result = true;

  template <typename T>
  void operator()(const char* /*member_name*/, const T& t1, const T& t2) {
    result = result && (t1 == t2);
  }
};

// general equality with visit_struct
template <typename T>
bool struct_eq(const T& t1, const T& t2) {
  eq_visitor vis;
  visit_struct::for_each(t1, t2, vis);
  return vis.result;
}

// convert visitable struct to json; supports types that can be assigned to
// nlohmann::json, such as strings, doubles, as well as recursive std::vector
// and visistable structs;
template <class Value>
auto visitable_to_json(const Value& value) {
  nlohmann::json result;
  if constexpr (visit_struct::traits::is_visitable<Value>::value) {
    // visitable struct --> visit each member and save as json object
    visit_struct::for_each(value, [&](const char* name, const auto& member) {
      result[name] = visitable_to_json(member);
    });
  } else if constexpr (is_vector_v<Value>) {
    // vector --> recursively save elements to json array
    for (const auto& elem : value) {
      result.push_back(visitable_to_json(elem));
    }
  } else if constexpr (wise_enum::is_wise_enum_v<Value>) {
    // enum --> save as string
    result = wise_enum::to_string(value);
  } else {
    // other basic value (string, number, ...)
    result = value;
  }
  return result;
}

}  // namespace rootba
