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

#include <functional>
#include <type_traits>

#include <nameof/nameof.hpp>
#include <wise_enum/wise_enum.h>

#include "rootba/options/enum_ref.hpp"
#include "rootba/util/assert.hpp"
#include "rootba/util/template_utils.hpp"

namespace rootba {

// EnumRef implmentation for wise_enum types
template <typename E, bool is_const>
class TWiseEnumRef : public TEnumRef<is_const> {
  static_assert(wise_enum::is_wise_enum_v<E>, "expected wise_enum type");

 public:
  using EnumType = E;
  using EnumValueType = mp::maybe_const<is_const, E>;
  using Type = TWiseEnumRef<EnumType, is_const>;
  using UnderlyingType = std::underlying_type_t<EnumType>;

  explicit TWiseEnumRef(EnumValueType& ref) : ref_(ref) {}

  static TEnumRefWrapper<is_const> make_wrapper(EnumValueType& ref) {
    return TEnumRefWrapper<is_const>(TWiseEnumRef(ref).clone_ref());
  }

  std::string enum_name() const override {
    return std::string(nameof::nameof_type<EnumType>());
  };

  int enum_size() const override {
    return static_cast<int>(wise_enum::size<EnumType>);
  };

  std::vector<std::string> get_possible_values() const override {
    std::vector<std::string> values;
    values.reserve(wise_enum::range<E>.size());
    for (auto e : wise_enum::range<E>) {
      values.emplace_back(wise_enum::to_string(e.value));
    }
    return values;
  }

  std::string get() const override {
    return std::string(wise_enum::to_string(ref_.get()));
  }

  int get_index() const override {
    auto begin = std::begin(wise_enum::range<EnumType>);
    auto end = std::end(wise_enum::range<EnumType>);
    auto it = std::find_if(begin, end,
                           [&](auto& e) { return e.value == ref_.get(); });
    if (it == end) {
      LOG(FATAL) << "Unreachable";
    }
    return it - begin;
  }

  bool set(std::string_view str) override {
    if constexpr (!is_const) {
      auto converted = wise_enum::from_string<EnumType>(str);
      if (!converted) {
        return false;
      }
      ref_.get() = *converted;
      return true;
    } else {
      ROOTBA_UNUSED(str);
      LOG(FATAL) << "unreachable";
    }
  }

  bool set_index(int index) override {
    if constexpr (!is_const) {
      if (index < 0 || index >= enum_size()) {
        return false;
      } else {
        ref_.get() = wise_enum::range<EnumType>[index].value;
        return true;
      }
    } else {
      LOG(FATAL) << "unreachable";
    }
  }

  std::unique_ptr<TEnumRef<is_const>> clone_ref() override {
    // make new ref by copy construction
    return std::make_unique<Type>(*this);
  }

 private:
  std::reference_wrapper<EnumValueType> ref_;
};

template <class E>
TEnumRefWrapper<std::is_const_v<E>> wrap_enum_ref(E& ref) {
  return TWiseEnumRef<std::decay_t<E>, std::is_const_v<E>>::make_wrapper(ref);
}

template <class E>
EnumConstRefWrapper wrap_enum_cref(const E& ref) {
  return TWiseEnumRef<E, false>::make_wrapper(ref);
}

// EnumValue implmentation for wise_enum types
template <typename E>
class WiseEnumValue : public EnumValue {
  static_assert(wise_enum::is_wise_enum_v<E>, "expected wise_enum type");

 public:
  using EnumType = E;
  using Type = WiseEnumValue<EnumType>;
  using UnderlyingType = std::underlying_type_t<EnumType>;

  explicit WiseEnumValue(EnumType value) : value_(value) {}

  static EnumValueWrapper make_wrapper(EnumType value) {
    return EnumValueWrapper(WiseEnumValue(value).clone());
  }

  std::string enum_name() const override {
    return std::string(nameof::nameof_type<EnumType>());
  };

  int enum_size() const override {
    return static_cast<int>(wise_enum::size<EnumType>);
  };

  std::vector<std::string> get_possible_values() const override {
    std::vector<std::string> values;
    values.reserve(wise_enum::range<EnumType>.size());
    for (auto e : wise_enum::range<EnumType>) {
      values.emplace_back(wise_enum::to_string(e.value));
    }
    return values;
  }

  int get_index() const override {
    auto begin = std::begin(wise_enum::range<EnumType>);
    auto end = std::end(wise_enum::range<EnumType>);
    auto it =
        std::find_if(begin, end, [&](auto& e) { return e.value == value_; });
    if (it == end) {
      LOG(FATAL) << "Unreachable";
    }
    return it - begin;
  }

  std::string get() const override {
    return std::string(wise_enum::to_string(value_));
  }

  bool set(std::string_view str) override {
    auto converted = wise_enum::from_string<EnumType>(str);
    if (!converted) {
      return false;
    }
    value_ = *converted;
    return true;
  }

  bool set_index(int index) override {
    if (index < 0 || index >= enum_size()) {
      return false;
    } else {
      value_ = wise_enum::range<EnumType>[index].value;
      return true;
    }
  }

  std::unique_ptr<EnumValue> clone() const override {
    // make new ref by copy construction
    return std::make_unique<Type>(*this);
  }

 private:
  EnumType value_;
};

template <class E>
EnumValueWrapper wrap_enum_value(const E& ref) {
  return WiseEnumValue<E>::make_wrapper(ref);
}

}  // namespace rootba
