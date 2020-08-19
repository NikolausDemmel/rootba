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

#include "rootba/util/stl_utils.hpp"

#include <gtest/gtest.h>

namespace rootba {

TEST(StlUtils, GetKeys) {
  EXPECT_EQ(get_keys(std::map<std::string, int>({{"foo", 23}, {"bar", 42}})),
            std::set<std::string>({"foo", "bar"}));
  EXPECT_EQ(get_keys(std::map<std::string, int>()), std::set<std::string>());
}

TEST(StlUtils, ToSet) {
  EXPECT_EQ(to_set(std::vector<int>({1, 2, 2, -1})), std::set<int>({-1, 1, 2}));
  EXPECT_EQ(to_set(std::vector<int>({})), std::set<int>({}));
}

TEST(StlUtils, IsSubsetOf) {
  EXPECT_TRUE(is_subset_of(std::set<int>({}), std::set<int>({3, 5})));
  EXPECT_TRUE(
      is_subset_of(std::set<int>({23, 42}), std::set<int>({1, 23, 42, 100})));
  EXPECT_TRUE(is_subset_of(std::set<int>({23, 42}), std::set<int>({23, 42})));
  EXPECT_FALSE(is_subset_of(std::set<int>({3, 5}), std::set<int>({})));
  EXPECT_FALSE(
      is_subset_of(std::set<int>({1, 23, 42, 100}), std::set<int>({23, 42})));
}

template <typename S>
class StlUtilsSetTest : public ::testing::Test {
 public:
  using Set = S;
};

using SetTypes = ::testing::Types<std::set<int>, std::unordered_set<int>>;
TYPED_TEST_SUITE(StlUtilsSetTest, SetTypes);

TYPED_TEST(StlUtilsSetTest, SetDifference) {
  using Set = typename TestFixture::Set;
  EXPECT_EQ(set_difference(Set({}), Set({})), Set({}));
  EXPECT_EQ(set_difference(Set({}), Set({23, 42})), Set({}));
  EXPECT_EQ(set_difference(Set({23, 42}), Set({1})), Set({23, 42}));
  EXPECT_EQ(set_difference(Set({23, 42}), Set({42})), Set({23}));
  EXPECT_EQ(set_difference(Set({23, 42}), Set({})), Set({23, 42}));
  EXPECT_EQ(set_difference(Set({23, 42}), Set({1, 23, 42, 100})), Set({}));
}

TYPED_TEST(StlUtilsSetTest, SetUnion) {
  using Set = typename TestFixture::Set;
  EXPECT_EQ(set_union(Set({}), Set({})), Set({}));
  EXPECT_EQ(set_union(Set({}), Set({23, 42})), Set({23, 42}));
  EXPECT_EQ(set_union(Set({23, 42}), Set({1})), Set({1, 23, 42}));
  EXPECT_EQ(set_union(Set({23, 42}), Set({42})), Set({23, 42}));
  EXPECT_EQ(set_union(Set({23, 42}), Set({})), Set({23, 42}));
  EXPECT_EQ(set_union(Set({23, 42}), Set({1, 23, 42, 100})),
            Set({1, 23, 42, 100}));
  EXPECT_EQ(set_union(Set({1, 42}), Set({23})), Set({1, 23, 42}));
}

TEST(StlUtilsSimpleSetTest, SetIntersections) {
  using Set = std::set<int>;
  EXPECT_EQ(set_intersection(Set({}), Set({})), Set({}));
  EXPECT_EQ(set_intersection(Set({}), Set({23, 42})), Set({}));
  EXPECT_EQ(set_intersection(Set({23, 42}), Set({1})), Set({}));
  EXPECT_EQ(set_intersection(Set({23, 42}), Set({42})), Set({42}));
  EXPECT_EQ(set_intersection(Set({23, 42}), Set({})), Set({}));
  EXPECT_EQ(set_intersection(Set({23, 42}), Set({1, 23, 42, 100})),
            Set({23, 42}));
  EXPECT_EQ(set_intersection(Set({1, 42}), Set({23})), Set({}));
}

TEST(StlUtils, EndsWith) {
  EXPECT_TRUE(ends_with("foobar", "foobar"));
  EXPECT_TRUE(ends_with("foobar", "bar"));
  EXPECT_TRUE(ends_with("foobar", ""));
  EXPECT_FALSE(ends_with("foobar", "foo"));
  EXPECT_FALSE(ends_with("foobar", "bazfoobar"));
  EXPECT_FALSE(ends_with("", "foobar"));
}

TEST(StlUtils, Sort) {
  std::vector<int> v0{};
  std::vector<int> v1{9, 3, 5};
  std::vector<int> v2{3, 5, 9};
  std::vector<std::string> v3{"foo", "bar", "baz"};
  std::vector<std::string> v4{"bar", "baz", "foo"};

  sort(v0);
  sort(v1);
  sort(v3);

  EXPECT_TRUE(v0.empty());
  EXPECT_TRUE(v1 == v2);
  EXPECT_TRUE(v3 == v4);
}

}  // namespace rootba
