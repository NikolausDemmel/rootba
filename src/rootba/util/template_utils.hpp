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
#pragma once

#include <tuple>
#include <type_traits>

// mp (meta programming) namespace to allow concise and short name that don't
// clutter the main namespace
namespace rootba::mp {

// ////////////////////////////////////////////////////////////////////////////
// overloads for generic lambdas
// See also: https://stackoverflow.com/q/55087826/1813258
template <class... Ts>
struct overload : Ts... {  // NOLINT
  using Ts::operator()...;
};
template <class... Ts>
overload(Ts...) -> overload<Ts...>;

// ////////////////////////////////////////////////////////////////////////////
// generic trait that is always false
template <class T>
struct always_false : std::integral_constant<bool, false> {};  // NOLINT

template <class T>
static constexpr bool always_false_v = always_false<T>::value;

// ////////////////////////////////////////////////////////////////////////////
// add const based on boolean
template <bool is_const, class T>
using maybe_const = std::conditional_t<is_const, const T, T>;

// ////////////////////////////////////////////////////////////////////////////
// Type trait is_instance for template instatiations
// See also: https://stackoverflow.com/a/61040973/1813258
namespace detail {
template <typename, template <typename...> typename>
struct is_instance_impl : public std::false_type {};

template <template <typename...> typename U, typename... Ts>
struct is_instance_impl<U<Ts...>, U> : public std::true_type {};
}  // namespace detail

template <typename T, template <typename...> typename U>
using is_instance = detail::is_instance_impl<std::decay_t<T>, U>;

template <typename T, template <typename...> typename U>
static constexpr bool is_instance_v = is_instance<T, U>::value;

}  // namespace rootba::mp
