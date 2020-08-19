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

#include <glog/logging.h>

namespace rootba {

// polymophic enum reference holder
template <bool is_const>
class TEnumRef {
 public:
  virtual ~TEnumRef() = default;

  // get underlying type name
  virtual std::string enum_name() const = 0;

  // get underlying type's number of values
  virtual int enum_size() const = 0;

  // get list of possible enum values
  virtual std::vector<std::string> get_possible_values() const = 0;

  // get the underlying value as string
  virtual std::string get() const = 0;

  // get the underlying value as index
  virtual int get_index() const = 0;

  // set the underlying value from string; (return false if invalid)
  virtual bool set(std::string_view str) = 0;

  // set the underlying value from index; (return false if invalid)
  virtual bool set_index(int index) = 0;

  // clone the reference (referencing the same underlying enum)
  virtual std::unique_ptr<TEnumRef> clone_ref() = 0;
};

// polymophic enum value holder
class EnumValue {
 public:
  virtual ~EnumValue() = default;

  // get underlying type name
  virtual std::string enum_name() const = 0;

  // get underlying type's number of values
  virtual int enum_size() const = 0;

  // get list of possible enum values
  virtual std::vector<std::string> get_possible_values() const = 0;

  // get the underlying value as string
  virtual std::string get() const = 0;

  // get the underlying value as index
  virtual int get_index() const = 0;

  // set the underlying value from string; (return false if invalid)
  virtual bool set(std::string_view str) = 0;

  // set the underlying value from index; (return false if invalid)
  virtual bool set_index(int index) = 0;

  // clone (make a copy)
  virtual std::unique_ptr<EnumValue> clone() const = 0;
};

// value type wrapper for EnumRef
template <bool is_const>
class TEnumRefWrapper {
 public:
  explicit TEnumRefWrapper(TEnumRef<is_const>& ref) : ref_(ref.clone_ref()) {
    // must always be non-null
    CHECK(ref_ != nullptr);
  }

  explicit TEnumRefWrapper(std::unique_ptr<TEnumRef<is_const>> ref)
      : ref_(std::move(ref)) {
    // must always be non-null
    CHECK(ref_ != nullptr);
  }

  TEnumRefWrapper(const TEnumRefWrapper& other)
      : ref_(other.ref_->clone_ref()) {}
  TEnumRefWrapper(TEnumRefWrapper&& other) noexcept = default;
  TEnumRefWrapper& operator=(const TEnumRefWrapper& other) {
    ref_ = other.ref_->clone_ref();
    return *this;
  }
  TEnumRefWrapper& operator=(TEnumRefWrapper&& other) noexcept = default;

  // get underlying type name
  std::string enum_name() const { return ref_->enum_name(); }

  // get underlying type's number of values
  int enum_size() const { return ref_->enum_size(); }

  // get list of possible enum values
  std::vector<std::string> get_possible_values() const {
    return ref_->get_possible_values();
  }

  // get the underlying value as string
  std::string get() const { return ref_->get(); }

  // get the underlying value as index
  int get_index() const { return ref_->get_index(); }

  // set the underlying value from string; (return false if invalid)
  bool set(std::string_view str) const { return ref_->set(str); }

  // set the underlying value from index; (return false if invalid)
  bool set_index(int index) const { return ref_->set_index(index); }

 private:
  // pointer to underlying polymorphic reference; may never be 0
  std::unique_ptr<TEnumRef<is_const>> ref_;
};

using EnumRefWrapper = TEnumRefWrapper<false>;
using EnumConstRefWrapper = TEnumRefWrapper<true>;

// value type wrapper for EnumValue
class EnumValueWrapper {
 public:
  explicit EnumValueWrapper(const EnumValue& value) : value_(value.clone()) {
    // must always be non-null
    CHECK(value_ != nullptr);
  }

  explicit EnumValueWrapper(std::unique_ptr<EnumValue> value)
      : value_(std::move(value)) {
    // must always be non-null
    CHECK(value_ != nullptr);
  }

  EnumValueWrapper(const EnumValueWrapper& other)
      : value_(other.value_->clone()) {}
  EnumValueWrapper(EnumValueWrapper&& other) = default;
  EnumValueWrapper& operator=(const EnumValueWrapper& other) {
    value_ = other.value_->clone();
    return *this;
  }
  EnumValueWrapper& operator=(EnumValueWrapper&& other) = default;

  // get underlying type name
  std::string enum_name() const { return value_->enum_name(); }

  // get underlying type's number of values
  int enum_size() const { return value_->enum_size(); }

  // get list of possible enum values
  std::vector<std::string> get_possible_values() const {
    return value_->get_possible_values();
  }

  // get the underlying value as string
  std::string get() const { return value_->get(); }

  // get the underlying value as index
  int get_index() const { return value_->get_index(); }

  // set the underlying value from string; (return false if invalid)
  bool set(std::string_view str) { return value_->set(str); }

  // set the underlying value from index; (return false if invalid)
  bool set_index(int index) { return value_->set_index(index); }

 private:
  // pointer to underlying polymorphic reference; may never be 0
  std::unique_ptr<EnumValue> value_;
};

}  // namespace rootba
