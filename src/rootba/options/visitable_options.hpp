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

#include <stdexcept>
#include <variant>

#include <visit_struct/visit_struct_intrusive.hpp>

#include "rootba/options/options_interface.hpp"
#include "rootba/options/wise_enum_ref.hpp"
#include "rootba/util/template_utils.hpp"

namespace rootba {

template <class Derived>
class VisitableOptions : public OptionsBase {
 private:
  // helper to avoid duplication of _accept code
  template <class ThisType, bool is_const_visitor>
  static void _accept_impl(ThisType* self,
                           TOptionsVisitor<is_const_visitor>& visitor) {
    static_assert(std::is_const_v<ThisType> == is_const_visitor);
    using DerivedType = mp::maybe_const<is_const_visitor, Derived>;
    visit_struct::for_each(
        static_cast<DerivedType&>(*self),
        mp::overload{
            // TODO: look into using expression sfinae... how to combine with
            // enable_if? How to improve error message?
            // See:
            // https://foonathan.net/blog/2015/11/30/overload-resolution-4.html
            // See:
            // https://vittorioromeo.info/index/blog/checking_expression_validity_in_place.html
            // See:
            // https://stackoverflow.com/a/30785453/1813258

            //[&](const char* name, auto& value) ->
            // std::void_t<decltype(visitor(name, std::ref(value)))>{
            //  visitor(name, std::ref(value));
            //},
            [&](const char* name,
                mp::maybe_const<is_const_visitor, bool>& value) {
              visitor(name, std::ref(value), self->_meta(name));
            },
            [&](const char* name,
                mp::maybe_const<is_const_visitor, int>& value) {
              visitor(name, std::ref(value), self->_meta(name));
            },
            [&](const char* name,
                mp::maybe_const<is_const_visitor, double>& value) {
              visitor(name, std::ref(value), self->_meta(name));
            },
            [&](const char* name,
                mp::maybe_const<is_const_visitor, std::string>& value) {
              visitor(name, std::ref(value), self->_meta(name));
            },
            [&](const char* name,
                mp::maybe_const<is_const_visitor, std::vector<int>>& value) {
              visitor(name, std::ref(value), self->_meta(name));
            },
            [&](const char* name,
                mp::maybe_const<is_const_visitor, std::vector<std::string>>&
                    value) {
              visitor(name, std::ref(value), self->_meta(name));
            },
            [&](const char* name, auto& child)
                -> std::enable_if_t<
                    std::is_base_of_v<OptionsInterface,
                                      std::decay_t<decltype(child)>>,
                    void> {
              // this is a nested option, so visit it
              visitor(name, child);
            },
            [&](const char* name, auto& child)
                -> std::enable_if_t<
                    wise_enum::is_wise_enum_v<std::decay_t<decltype(child)>>,
                    void> {
              // wise enum we need to pack in a generic enum holder
              visitor(name, wrap_enum_ref(child), self->_meta(name));
            },
            [&](const char* name, auto& child)
                -> std::enable_if_t<is_wise_enum_flags_type<
                                        std::decay_t<decltype(child)>>::value,
                                    void> {
              // wise enum flags: we need to pack in a generic flags holder -->
              // as reference (not copy)
              visitor(name, wrap_flags_ref(child), self->_meta(name));
            },
            [&](const char* name, auto& child)
                -> std::enable_if_t<
                    !std::is_base_of_v<OptionsInterface,
                                       std::decay_t<decltype(child)>> &&
                        !wise_enum::is_wise_enum_v<
                            std::decay_t<decltype(child)>> &&
                        !is_wise_enum_flags_type<
                            std::decay_t<decltype(child)>>::value,
                    void> {
              ROOTBA_UNUSED(name);
              // catch all --> provide better compile error with this assert
              static_assert(mp::always_false_v<std::decay_t<decltype(child)>>,
                            "support for options variable of this type not "
                            "(yet) implemented");

              throw std::runtime_error("unreachable");
            }});
  }

 public:  // Options interface
  void _accept(OptionsVisitor& visitor) override {
    _accept_impl(this, visitor);
  }
  void _accept(ConstOptionsVisitor& visitor) const override {
    _accept_impl(this, visitor);
  }

 protected:
  VisitableOptions() = default;

  virtual const OptionsMeta& _meta_default() const {
    static OptionsMeta default_val;
    return default_val;
  }

  const OptionsMeta& _meta(const std::string& name) const {
    if (const auto it = _meta_map.find(name); it != _meta_map.end()) {
      return it->second;
    } else {
      return _meta_default();
    }
  }

  OptionsMeta& _register_meta(const std::string& name, OptionsMeta meta_value) {
    const auto& [it, inserted] = _meta_map.emplace(name, std::move(meta_value));
    CHECK(inserted);
    return it->second;
  }

  std::map<std::string, OptionsMeta> _meta_map;
};

#define VISITABLE_OPTIONS_DEFAULT_META(INITIALIZER)           \
  virtual const OptionsMeta& _meta_default() const override { \
    static OptionsMeta default_val =                          \
        OptionsMetaInitializer<bool>().INITIALIZER.value;     \
    return default_val;                                       \
  }                                                           \
  static_assert(true)

#define VISITABLE_META(TYPE, NAME, META_INITIALIZER)                      \
  VISITABLE_INIT(                                                         \
      TYPE, NAME,                                                         \
      _register_meta(#NAME, OptionsMetaInitializer<TYPE>(_meta_default()) \
                                .init_default_construct()                 \
                                .META_INITIALIZER.value)                  \
          .get_default_value<TYPE>())

#define VISITABLE_META_DEFAULT(TYPE, NAME) VISITABLE_META(TYPE, NAME, noop())

}  // namespace rootba
