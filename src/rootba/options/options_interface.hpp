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
#include <map>
#include <string_view>
#include <variant>

#include <nlohmann/json_fwd.hpp>

#include "rootba/options/enum_ref.hpp"
#include "rootba/options/flags_ref.hpp"
#include "rootba/options/wise_enum_ref.hpp"
#include "rootba/util/enum_utils.hpp"
#include "rootba/util/template_utils.hpp"
#include "rootba/util/typesafe_utils.hpp"

namespace rootba {

// TODO: use ts::object_ref instead of std::reference_wrapper
// variant type of all supported types for option variables
template <bool is_const>
using TOptionsVariableRef = std::variant<
    std::reference_wrapper<mp::maybe_const<is_const, bool>>,
    std::reference_wrapper<mp::maybe_const<is_const, int>>,
    std::reference_wrapper<mp::maybe_const<is_const, double>>,
    std::reference_wrapper<mp::maybe_const<is_const, std::string>>,
    std::conditional_t<is_const, EnumConstRefWrapper, EnumRefWrapper>,
    std::conditional_t<is_const, FlagsConstWrapper, FlagsWrapper>,
    std::reference_wrapper<mp::maybe_const<is_const, std::vector<int>>>,
    std::reference_wrapper<
        mp::maybe_const<is_const, std::vector<std::string>>>>;

using OptionsVariableRef = TOptionsVariableRef<false>;
using OptionsVariableConstRef = TOptionsVariableRef<true>;

using OptionsVariableValue =
    std::variant<bool, int, double, std::string, EnumValueWrapper, FlagsWrapper,
                 std::vector<int>, std::vector<std::string>>;

WISE_ENUM_CLASS(OptionsMetaFlag, (READ_ONLY, 1 << 0), (HIDE_GUI, 1 << 1),
                (HIDE_CLI, 1 << 2));
ROOTBA_DECLARE_FLAGS(OptionsMetaFlags, OptionsMetaFlag);

struct OptionsMeta {
  double range[2] = {-std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::infinity()};
  std::string help;
  OptionsMetaFlags flags{flags::empty};
  bool logscale = false;
  OptionsVariableValue default_value;

  template <typename T>
  T get_default_value() {
    if constexpr (wise_enum::is_wise_enum_v<T>) {
      auto wrapper = std::get<EnumValueWrapper>(default_value);
      wise_enum::optional_type<T> value =
          wise_enum::from_string<T>(wrapper.get());
      CHECK(value);  // invalid value is a programming error
      return *value;
    } else if constexpr (is_wise_enum_flags_type<T>::value) {
      auto wrapper = std::get<FlagsWrapper>(default_value);
      T value{flags::empty};
      for (const auto& s : wrapper.get()) {
        auto converted = wise_enum::from_string<typename T::enum_type>(s);
        CHECK(converted);  // invalid value is a programming error
        value |= *converted;
      }
      return value;
    } else {
      return std::get<T>(default_value);
    }
  }
};

template <typename T>
struct OptionsMetaInitializer {
  OptionsMeta value;

  OptionsMetaInitializer() = default;
  explicit OptionsMetaInitializer(const OptionsMeta& initial_value)
      : value(initial_value) {}

  OptionsMetaInitializer& help(std::string text) {
    value.help = std::move(text);
    return *this;
  }

  OptionsMetaInitializer& range(double min, double max) {
    value.range[0] = min;
    value.range[1] = max;
    return *this;
  }

  OptionsMetaInitializer& logscale(bool val = true) {
    value.logscale = val;
    return *this;
  }

  OptionsMetaInitializer& readonly(bool val = true) {
    value.flags |= OptionsMetaFlag::READ_ONLY;
    if (!val) {
      value.flags ^= OptionsMetaFlag::READ_ONLY;
    }
    return *this;
  }

  OptionsMetaInitializer& hidegui(bool val = true) {
    value.flags |= OptionsMetaFlag::HIDE_GUI;
    if (!val) {
      value.flags ^= OptionsMetaFlag::HIDE_GUI;
    }
    return *this;
  }

  OptionsMetaInitializer& hidecli(bool val = true) {
    value.flags |= OptionsMetaFlag::HIDE_CLI;
    if (!val) {
      value.flags ^= OptionsMetaFlag::HIDE_CLI;
    }
    return *this;
  }

  OptionsMetaInitializer& init(T val) {
    if constexpr (wise_enum::is_wise_enum_v<T>) {
      value.default_value = wrap_enum_value(val);
    } else if constexpr (is_wise_enum_flags_type<T>::value) {
      value.default_value = FlagsWrapper(val);
    } else {
      value.default_value = std::move(val);
    }
    return *this;
  }

  OptionsMetaInitializer& init_default_construct() {
    if constexpr (wise_enum::is_wise_enum_v<T>) {
      T default_value = std::begin(wise_enum::range<T>)->value;
      value.default_value = wrap_enum_value(default_value);
    } else if constexpr (is_wise_enum_flags_type<T>::value) {
      value.default_value = FlagsWrapper(T{flags::empty});
    } else {
      value.default_value = T{};
    }
    return *this;
  }

  OptionsMetaInitializer& noop() { return *this; }
};

class OptionsInterface;

template <bool is_const_visitor>
class TOptionsVisitor {
 public:
  using OptionsVariableRefType =
      std::conditional_t<is_const_visitor, OptionsVariableConstRef,
                         OptionsVariableRef>;
  using OptionsInterfaceType =
      std::conditional_t<is_const_visitor, const OptionsInterface,
                         OptionsInterface>;

  virtual inline ~TOptionsVisitor() = default;
  virtual void operator()(const char* name, OptionsVariableRefType var,
                          const OptionsMeta& meta) = 0;
  virtual void operator()(const char* name, OptionsInterfaceType& child) = 0;
};

using OptionsVisitor = TOptionsVisitor<false>;
using ConstOptionsVisitor = TOptionsVisitor<true>;

class OptionsInterface {
 public:
  virtual inline ~OptionsInterface() = default;

  virtual void _accept(OptionsVisitor& visitor) = 0;
  virtual void _accept(ConstOptionsVisitor& visitor) const = 0;

  struct LoadConfigOptions {
    // workaround: default constructor causes compile error... compiler bug?
    LoadConfigOptions() {}  // NOLINT
    std::string prefix;
    bool missing_config_file_is_error = true;
    bool check_unused = true;
    std::vector<std::string> allow_unused_top_level;
  };

  virtual bool _load(
      const nlohmann::json& json_data,
      const LoadConfigOptions& options = LoadConfigOptions()) = 0;
  virtual bool _load(
      const std::string& filename,
      const LoadConfigOptions& options = LoadConfigOptions()) = 0;

  // TODO: add more convenience functions to directly access member values and
  // meta by name

  virtual void _print(std::ostream& os) const = 0;
};

std::ostream& operator<<(std::ostream& os, const OptionsInterface& options);

class OptionsBase : public OptionsInterface {
 public:
  bool _load(const nlohmann::json& json_data,
             const LoadConfigOptions& options = LoadConfigOptions()) override;
  bool _load(const std::string& filename,
             const LoadConfigOptions& options = LoadConfigOptions()) override;

  void _print(std::ostream& os) const override;
};

}  // namespace rootba
