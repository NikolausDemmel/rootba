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

#include <sstream>

#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <gtest/gtest.h>

namespace rootba {

// helper for compile tests and generic function test of cereal serialization
// We use this to test that headers implementing cereal serializations have all
// the needed includes, since the compiler error messages for some missing
// includes are not so helpful to track down which include is missing.

// It checks that the serialized representation is invariant to a
// deserialize-serialize cycle.

template <class T, class InputArchive>
void test_io_load(const T& prototype) {
  std::stringstream ss;

  // don't actualy execute the load, since in general we don't have data
  // available for this general test
  T obj(prototype);
  if (false) {  // NOLINT
    InputArchive archive(ss);
    archive(obj);
  }

  SUCCEED();
}

template <class T, class OutputArchive, class InputArchive>
void test_io_cycle(const T& prototype) {
  std::stringstream ss;

  T obj(prototype);

  {
    OutputArchive archive(ss);
    archive(obj);
  }

  // What if no default constructor is available?
  // T obj2(prototype);
  T obj2;

  {
    InputArchive archive(ss);
    archive(obj2);
  }

  std::stringstream ss2;

  {
    OutputArchive archive(ss2);
    archive(obj2);
  }

  // Binary representation after deserialize-serialize should be the same
  EXPECT_EQ(ss.str(), ss2.str());
}

template <class T, typename... Args>
void test_io_impl(Args&&... args) {
  SCOPED_TRACE("");
  const T obj(std::forward<Args>(args)...);
  test_io_cycle<T, cereal::JSONOutputArchive, cereal::JSONInputArchive>(obj);
  test_io_cycle<T, cereal::BinaryOutputArchive, cereal::BinaryInputArchive>(
      obj);
}

template <class T, typename... Args>
void test_io_load_impl(Args&&... args) {
  SCOPED_TRACE("");
  const T obj(std::forward<Args>(args)...);
  test_io_load<T, cereal::JSONInputArchive, Args...>(obj);
  test_io_load<T, cereal::BinaryInputArchive, Args...>(obj);
}

}  // namespace rootba
