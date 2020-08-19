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

#include <type_traits>

///////////////////////////////////////////////////////////////////////////////

#include <wise_enum/wise_enum.h>

///////////////////////////////////////////////////////////////////////////////

#include <flags/flags.hpp>

// helper to declare flags for enums in rootba namespace
#define ROOTBA_DECLARE_FLAGS(FLAGTYPE, ENUMTYPE) \
  }                                              \
  ALLOW_FLAGS_FOR_ENUM(rootba::ENUMTYPE);        \
  namespace rootba {                             \
  using FLAGTYPE = flags::flags<ENUMTYPE>

#define ROOTBA_DECLARE_CLASS_FLAGS(FLAGTYPE, ENUMTYPE) \
  using FLAGTYPE = flags::flags<ENUMTYPE>

#define ROOTBA_REGISTER_CLASS_FLAGS(ENUMTYPE) \
  }                                           \
  ALLOW_FLAGS_FOR_ENUM(rootba::ENUMTYPE);     \
  namespace rootba {                          \
  static_assert(true)

///////////////////////////////////////////////////////////////////////////////

namespace rootba {

// "all flags" for wise-enum supported flag types
template <class FlagsType>
constexpr FlagsType  // NOLINT(readability-const-return-type)
compute_all_flags_union() {
  // Note: beware of https://github.com/grisumbras/enum-flags/issues/15
  // which is why we cannot do simply "~FlagsType(flags::empty)"
  auto res = std::decay_t<FlagsType>(flags::empty);
  for (const auto& [value, _] :
       wise_enum::range<typename std::decay_t<FlagsType>::enum_type>) {
    res = res | value;
  }
  return res;
}

template <typename FlagsType>
constexpr FlagsType no_flags = FlagsType(flags::empty);

template <typename FlagsType>
constexpr FlagsType all_flags = compute_all_flags_union<FlagsType>();

// type trait for flags
template <class F>
struct is_flags_type : std::false_type {};

template <class E>
struct is_flags_type<flags::flags<E>> : std::true_type {};

// type trait for flags of wise-enums
template <class F>
struct is_wise_enum_flags_type : std::false_type {};

template <class E>
struct is_wise_enum_flags_type<flags::flags<E>>
    : std::enable_if_t<wise_enum::is_wise_enum_v<E>, std::true_type> {};

}  // namespace rootba
