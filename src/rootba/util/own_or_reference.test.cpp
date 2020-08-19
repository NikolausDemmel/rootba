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

#include "rootba/util/own_or_reference.hpp"

#include <gtest/gtest.h>

namespace rootba {

TEST(OwnOrReference, Construction) {
  std::string foo("foo");
  std::string bar("bar");

  // non const
  {
    OwnOrReference<std::string> oor;
    *oor = bar;
    EXPECT_TRUE(oor.owned());
    EXPECT_EQ(*oor, bar);

    OwnOrReference<std::string> oor2(&foo);
    EXPECT_FALSE(oor2.owned());
    EXPECT_EQ(oor2.get(), &foo);

    OwnOrReference<std::string> oor3(oor);
    EXPECT_TRUE(oor3.owned());
    EXPECT_EQ(*oor3, bar);
    EXPECT_NE(oor3.get(), oor.get());

    OwnOrReference<std::string> oor4(std::move(oor));
    EXPECT_TRUE(oor4.owned());
    EXPECT_EQ(*oor4, bar);

    OwnOrReference<std::string> oor5(oor2);
    EXPECT_FALSE(oor5.owned());
    EXPECT_EQ(oor5.get(), &foo);

    OwnOrReference<std::string> oor6(std::move(oor2));
    EXPECT_FALSE(oor6.owned());
    EXPECT_EQ(oor6.get(), &foo);

    OwnOrReference<std::string> oor7(nullptr);
    *oor7 = bar;
    EXPECT_TRUE(oor7.owned());
    EXPECT_EQ(*oor7, bar);
  }

  // const
  {
    OwnOrReference<const std::string> oor;
    EXPECT_TRUE(oor.owned());
    EXPECT_EQ(*oor, std::string());

    OwnOrReference<const std::string> oor2(&foo);
    EXPECT_FALSE(oor2.owned());
    EXPECT_EQ(oor2.get(), &foo);

    OwnOrReference<const std::string> oor3(oor);
    EXPECT_TRUE(oor3.owned());
    EXPECT_EQ(*oor3, std::string());
    EXPECT_NE(oor3.get(), oor.get());

    OwnOrReference<const std::string> oor4(std::move(oor));
    EXPECT_TRUE(oor4.owned());
    EXPECT_EQ(*oor4, std::string());

    OwnOrReference<const std::string> oor5(oor2);
    EXPECT_FALSE(oor5.owned());
    EXPECT_EQ(oor5.get(), &foo);

    OwnOrReference<const std::string> oor6(std::move(oor2));
    EXPECT_FALSE(oor6.owned());
    EXPECT_EQ(oor6.get(), &foo);

    OwnOrReference<const std::string> oor7(nullptr);
    EXPECT_TRUE(oor7.owned());
    EXPECT_EQ(*oor7, std::string());
  }
}

TEST(OwnOrReference, CopyAssignment) {
  // non const
  {
    std::string foo("foo");
    std::string bar("bar");
    std::string quux("quux");
    std::string quax("quax");

    OwnOrReference<std::string> oor;
    *oor = foo;
    EXPECT_TRUE(oor.owned());
    EXPECT_EQ(*oor, foo);

    OwnOrReference<std::string> oor2;
    *oor2 = bar;
    EXPECT_TRUE(oor2.owned());
    EXPECT_EQ(*oor2, bar);

    OwnOrReference<std::string> oor3(&quux);
    EXPECT_FALSE(oor3.owned());
    EXPECT_EQ(oor3.get(), &quux);

    OwnOrReference<std::string> oor4(&quax);
    EXPECT_FALSE(oor4.owned());
    EXPECT_EQ(oor4.get(), &quax);

    // both owned
    oor = oor2;
    EXPECT_TRUE(oor.owned());
    EXPECT_EQ(*oor, *oor2);
    EXPECT_NE(oor.get(), oor2.get());

    // lhs owned, rhs not owned
    oor = oor3;
    EXPECT_FALSE(oor.owned());
    EXPECT_EQ(*oor, *oor3);
    EXPECT_EQ(oor.get(), oor3.get());

    // lhs not owned, rhs owned
    oor3 = oor2;
    EXPECT_FALSE(oor3.owned());
    EXPECT_EQ(*oor3, bar);
    EXPECT_EQ(quux, bar);
    EXPECT_NE(oor3.get(), oor2.get());

    // lhs not owned, rhs not owned
    oor3 = oor4;
    EXPECT_FALSE(oor3.owned());
    EXPECT_EQ(*oor3, quax);
    EXPECT_NE(quux, quax);
    EXPECT_EQ(oor3.get(), oor4.get());
  }

  // const
  {
    std::string quux("quux");
    std::string quax("quax");

    OwnOrReference<std::string> oor3(&quux);
    EXPECT_FALSE(oor3.owned());
    EXPECT_EQ(oor3.get(), &quux);

    OwnOrReference<std::string> oor4(&quax);
    EXPECT_FALSE(oor4.owned());
    EXPECT_EQ(oor4.get(), &quax);

    // lhs not owned, rhs not owned
    oor3 = oor4;
    EXPECT_FALSE(oor3.owned());
    EXPECT_EQ(*oor3, quax);
    EXPECT_NE(quux, quax);
    EXPECT_EQ(oor3.get(), oor4.get());
  }
}

TEST(OwnOrReference, MoveAssignment) {
  // non const
  {
    std::string foo("foo");
    std::string bar("bar");
    std::string quux("quux");
    std::string quax("quax");

    OwnOrReference<std::string> oor;
    *oor = foo;
    EXPECT_TRUE(oor.owned());
    EXPECT_EQ(*oor, foo);

    OwnOrReference<std::string> oor2;
    *oor2 = bar;
    EXPECT_TRUE(oor2.owned());
    EXPECT_EQ(*oor2, bar);

    OwnOrReference<std::string> oor3(&quux);
    EXPECT_FALSE(oor3.owned());
    EXPECT_EQ(oor3.get(), &quux);

    OwnOrReference<std::string> oor4(&quax);
    EXPECT_FALSE(oor4.owned());
    EXPECT_EQ(oor4.get(), &quax);

    // both owned
    oor = std::move(oor2);
    EXPECT_TRUE(oor.owned());
    EXPECT_EQ(*oor, bar);

    // lhs owned, rhs not owned
    oor = std::move(oor3);
    EXPECT_FALSE(oor.owned());
    EXPECT_EQ(oor.get(), &quux);

    // lhs not owned, rhs owned
    oor2.reset();
    *oor2 = bar;
    oor3 = std::move(oor2);
    EXPECT_FALSE(oor3.owned());
    EXPECT_EQ(*oor3, bar);
    EXPECT_EQ(quux, bar);
    EXPECT_NE(oor3.get(), &bar);

    // lhs not owned, rhs not owned
    oor3 = std::move(oor4);
    EXPECT_FALSE(oor3.owned());
    EXPECT_EQ(*oor3, quax);
    EXPECT_NE(quux, quax);
    EXPECT_EQ(oor3.get(), oor4.get());
  }

  // const
  {
    std::string quux("quux");
    std::string quax("quax");

    OwnOrReference<std::string> oor3(&quux);
    EXPECT_FALSE(oor3.owned());
    EXPECT_EQ(oor3.get(), &quux);

    OwnOrReference<std::string> oor4(&quax);
    EXPECT_FALSE(oor4.owned());
    EXPECT_EQ(oor4.get(), &quax);

    // lhs not owned, rhs not owned
    oor3 = std::move(oor4);
    EXPECT_FALSE(oor3.owned());
    EXPECT_EQ(*oor3, quax);
    EXPECT_NE(quux, quax);
    EXPECT_EQ(oor3.get(), oor4.get());
  }
}

}  // namespace rootba
