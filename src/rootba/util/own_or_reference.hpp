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

#include <glog/logging.h>

namespace rootba {

// OwnOrReference behaves like a smart pointer, but it can never be null (unless
// it's in a "moved from" state). By default, it holds its own copy of an object
// and makes sure to release memory upon destruction. But we can also assign it
// an external (not null) pointer and, which the reference instead without any
// ownership.
//
// It is the user's responsibility to ensure the lifetime of the external object
// doesn't end before the last use via OwnOrReference.
//
// Copy semantics are like for pointers when an external refernece is set,
// otherwise it's more like value semantics, as we create a new copy of the
// owned object.
//
// Accessors are the usual * and -> operators, as well as get() for the
// underlying pointer. Constness of OwnOrReference extends to the pointed-to
// element.
//
// The template type T must be default constructible.
template <class T>
class OwnOrReference {
 public:
  using ObjType = std::remove_cv_t<T>;

 public:
  OwnOrReference() = default;
  explicit OwnOrReference(T* external) {
    if (external == nullptr) {
      ptr_ = new ObjType();
      owned_ = true;
    } else {
      ptr_ = external;
      owned_ = false;
    }
  }
  ~OwnOrReference() { free(); }

  // copy & move constructors
  OwnOrReference(const OwnOrReference<T>& other) {
    if (other.owned()) {
      // copy-construct object
      ptr_ = new ObjType(*other.ptr_);
      owned_ = true;
    } else {
      set(other.ptr_);
    }
  }
  OwnOrReference(OwnOrReference<T>&& other) noexcept {
    if (other.owned()) {
      // move the object from other
      ptr_ = other.ptr_;
      other.ptr_ = nullptr;
      owned_ = true;
    } else {
      set(other.ptr_);
    }
  }

  // copy & move assignment
  OwnOrReference& operator=(const OwnOrReference<T>& other) {
    if (other.owned()) {
      if (!ptr_) {
        // deal with moved-from state
        ptr_ = new ObjType;
      }
      *ptr_ = *other.ptr_;
    } else {
      CHECK_NOTNULL(ptr_);
      set(other.ptr_);
    }
    return *this;
  }
  OwnOrReference& operator=(OwnOrReference<T>&& other) noexcept {
    if (other.owned()) {
      if (owned()) {
        // both owned: swap ptrs
        std::swap(ptr_, other.ptr_);
      } else {
        // other owned, ours not -> move-assign object
        *ptr_ = std::move(*other.ptr_);
      }
    } else {
      CHECK_NOTNULL(ptr_);
      set(other.ptr_);
    }
    return *this;
  }

  void reset() {
    free();
    ptr_ = new ObjType;
    owned_ = true;
  }

  void set(T* external) {
    free();
    ptr_ = external;
    owned_ = false;
  }

  const T* get() const { return ptr_; }
  T* get() { return ptr_; }

  const T& operator*() const { return *ptr_; }
  T& operator*() { return *ptr_; }

  const T* operator->() const { return ptr_; }
  T* operator->() { return ptr_; }

  bool owned() const { return owned_; }

 private:
  void free() {
    if (owned_) {
      // this is fine even if called repeatedly or after moving-from this, since
      // deleting a nullptr is a noop
      delete ptr_;
      ptr_ = nullptr;
    }
  }

 private:
  T* ptr_ = new ObjType;
  bool owned_ = true;
};

}  // namespace rootba
