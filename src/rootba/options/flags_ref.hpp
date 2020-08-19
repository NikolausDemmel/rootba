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

#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include <glog/logging.h>
#include <nameof/nameof.hpp>

#include "rootba/util/assert.hpp"
#include "rootba/util/enum_utils.hpp"
#include "rootba/util/template_utils.hpp"

namespace rootba {

template <bool is_const>
class AbstractFlags {
 public:
  using Values = std::vector<std::string>;

  virtual ~AbstractFlags() = default;

  /// get name of underlying enum type
  virtual std::string enum_name() const = 0;

  /// get list of currently set flag values
  virtual Values get() const = 0;

  /// set falgs from list of strings; (return false if ony are invalid)
  virtual bool set(const Values& values) = 0;

  /// get list of possible flag values
  virtual Values get_possible_flags() const = 0;

  /// clone wrapper object (copy for value, new ref to same object for
  /// reference)
  virtual std::unique_ptr<AbstractFlags> clone() const = 0;
};

template <class E, bool is_const,
          class = std::enable_if<wise_enum::is_wise_enum_v<E>>>
class WiseEnumFlagsRef : public AbstractFlags<is_const> {
 public:
  using EnumType = E;
  using FlagsType = mp::maybe_const<is_const, flags::flags<EnumType>>;
  using Values = typename AbstractFlags<is_const>::Values;

  explicit WiseEnumFlagsRef(FlagsType& ref) : ref_(ref) {}

  std::string enum_name() const override {
    return std::string(nameof::nameof_type<EnumType>());
  }

  Values get() const override {
    Values current;
    for (auto e : ref_.get()) {
      current.push_back(std::string(wise_enum::to_string(e)));
    }
    return current;
  }

  bool set(const Values& values) override {
    if constexpr (!is_const) {
      bool ok = true;
      auto& f = ref_.get();
      f.clear();
      for (const auto& s : values) {
        auto converted = wise_enum::from_string<EnumType>(s);
        if (!converted) {
          ok = false;
          continue;
        }
        f |= *converted;
      }
      return ok;
    } else {
      ROOTBA_UNUSED(values);
      LOG(FATAL) << "unreachable";
    }
  }

  Values get_possible_flags() const override {
    Values all;
    for (auto e : all_flags<FlagsType>) {
      all.push_back(std::string(wise_enum::to_string(e)));
    }
    return all;
  }

  std::unique_ptr<AbstractFlags<is_const>> clone() const override {
    return std::make_unique<WiseEnumFlagsRef<EnumType, is_const>>(ref_.get());
  }

 private:
  std::reference_wrapper<FlagsType> ref_;
};

template <class E>
class WiseEnumFlagsValue : public AbstractFlags<false> {
 public:
  using EnumType = E;
  using FlagsType = flags::flags<EnumType>;

  explicit WiseEnumFlagsValue(FlagsType value) : value_(value) {}

  std::string enum_name() const override {
    return std::string(nameof::nameof_type<EnumType>());
  }

  Values get() const override {
    Values current;
    for (auto e : value_) {
      current.push_back(std::string(wise_enum::to_string(e)));
    }
    return current;
  }

  bool set(const Values& values) override {
    bool ok = true;
    value_.clear();
    for (const auto& s : values) {
      auto converted = wise_enum::from_string<EnumType>(s);
      if (!converted) {
        ok = false;
        continue;
      }
      value_ |= *converted;
    }
    return ok;
  }

  Values get_possible_flags() const override {
    Values all;
    for (auto e : all_flags<FlagsType>) {
      all.push_back(std::string(wise_enum::to_string(e)));
    }
    return all;
  }

  std::unique_ptr<AbstractFlags> clone() const override {
    return std::make_unique<WiseEnumFlagsValue>(value_);
  }

 private:
  FlagsType value_;
};

template <bool is_const>
class TFlagsWrapper {
 public:
  using Values = typename AbstractFlags<is_const>::Values;

  // create value flags wrapper
  template <class E>
  explicit TFlagsWrapper(flags::flags<E> value)
      : ptr_(std::make_unique<WiseEnumFlagsValue<E>>(value)) {}

  // create reference flags wrapper
  template <class F>
  explicit TFlagsWrapper(std::reference_wrapper<F> ref) {
    static_assert(is_wise_enum_flags_type<std::decay_t<F>>::value);
    static_assert(std::is_const_v<F> == is_const);
    using EnumType = typename F::enum_type;
    ptr_ = std::make_unique<WiseEnumFlagsRef<EnumType, is_const>>(ref);
  }

  TFlagsWrapper(const TFlagsWrapper& other) : ptr_(other.ptr_->clone()) {}
  TFlagsWrapper(TFlagsWrapper&& other) noexcept = default;
  TFlagsWrapper& operator=(const TFlagsWrapper& other) {
    ptr_ = other.ptr_->clone();
    return *this;
  }
  TFlagsWrapper& operator=(TFlagsWrapper&& other) noexcept = default;

  std::string enum_name() const { return ptr_->enum_name(); }

  Values get() const { return ptr_->get(); }

  bool set(const Values& values) const {
    static_assert(!is_const);
    return ptr_->set(values);
  }

  Values get_possible_flags() const { return ptr_->get_possible_flags(); }

 private:
  std::unique_ptr<AbstractFlags<is_const>> ptr_;
};

using FlagsWrapper = TFlagsWrapper<false>;
using FlagsConstWrapper = TFlagsWrapper<true>;

template <class E>
FlagsWrapper wrap_flags_ref(flags::flags<E>& ref) {
  return FlagsWrapper(std::ref(ref));
}

template <class E>
FlagsConstWrapper wrap_flags_ref(const flags::flags<E>& ref) {
  return FlagsConstWrapper(std::ref(ref));
}

template <class E>
FlagsWrapper wrap_flags_cref(const flags::flags<E>& ref) {
  return FlagsWrapper(std::ref(ref));
}

}  // namespace rootba
