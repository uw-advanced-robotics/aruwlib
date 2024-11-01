/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>

#include "tap/units/quantity.hpp"

using namespace tap::units;

TEST(Quantity, constructors__default_value)
{
    Quantity<> q1;
    EXPECT_FLOAT_EQ(0, q1.valueOf());
    Quantity<> q2(5);
    EXPECT_FLOAT_EQ(5, q2.valueOf());
}

TEST(Quantity, constructor__copy)
{
    Quantity<> q1(5);
    Quantity<> q2(q1);
    EXPECT_FLOAT_EQ(5, q2.valueOf());
    q1 = Quantity<>(10);
    EXPECT_FLOAT_EQ(5, q2.valueOf());
}

TEST(Quantity, covertTo) {
    Quantity<ratio<1>, ratio<1>> q1(5);
    Quantity<ratio<1>, ratio<1>, ratio<0>, ratio<0>, ratio<0>, ratio<0>, 1> q2(10);
    EXPECT_FLOAT_EQ(0.5, q1.convertTo(q2));
}

TEST(Quantity, operator__add_subtract_equals)
{
    Quantity<> q1(5);
    Quantity<> q2(10);
    q1 += q2;
    EXPECT_FLOAT_EQ(15, q1.valueOf());

    q1 -= q2;
    EXPECT_FLOAT_EQ(5, q1.valueOf());

    q1 += Quantity<>(0);
    EXPECT_FLOAT_EQ(5, q1.valueOf());

    q1 -= Quantity<>(0);
    EXPECT_FLOAT_EQ(5, q1.valueOf());
}

TEST(Quantity, operator__multiply_divide_equals)
{
    Quantity<> q1(5);
    q1 *= 3;
    EXPECT_FLOAT_EQ(15, q1.valueOf());

    q1 /= 5;
    EXPECT_FLOAT_EQ(3, q1.valueOf());

    q1 *= 1;
    EXPECT_FLOAT_EQ(3, q1.valueOf());

    q1 /= 1;
    EXPECT_FLOAT_EQ(3, q1.valueOf());

    q1 *= 0;
    EXPECT_FLOAT_EQ(0, q1.valueOf());
}

TEST(Quantity, operator__add_subtract)
{
    Quantity<ratio<1>> q1(5);
    Quantity<ratio<1>> q2(10);
    Quantity<ratio<1>> q3 = q1 + q2;

    EXPECT_FLOAT_EQ(15, q3.valueOf());

    q3 = q1 - q2;
    EXPECT_FLOAT_EQ(-5, q3.valueOf());
}

TEST(Quantity, operator__scalar_multiply_divide) {}

TEST(Quantity, operator__quantity_multiply_divide)
{
    Quantity<ratio<1>> q1(5);
    Quantity<ratio<1>, ratio<1>> q2(10);

    Quantity<ratio<2>, ratio<1>> q3 = q1 * q2;
    EXPECT_FLOAT_EQ(50, q3.valueOf());

    Quantity<ratio<0>, ratio<-1>> q4 = q1 / q2;
    EXPECT_FLOAT_EQ(0.5, q4.valueOf());
}

TEST(Quantity, frame__inOtherFrame)
{
    Quantity<ratio<1>, ratio<1>> q1(5);
    Quantity<ratio<1>, ratio<1>, ratio<0>, ratio<0>, ratio<0>, ratio<0>, 1> q2 =
        q1.inOtherFrame<1>();
    EXPECT_FLOAT_EQ(5, q2.valueOf());
    Quantity<ratio<1>, ratio<1>, ratio<0>, ratio<0>, ratio<0>, ratio<0>, 1> q3 =
        q1.inOtherFrame<1>(2);
    EXPECT_FLOAT_EQ(10, q3.valueOf());
}

TEST(Quantity, concept__isQuantity) {
    constexpr bool a = isQuantity<Quantity<>>;
    EXPECT_TRUE(a);
    constexpr bool b = isQuantity<Quantity<ratio<1>, ratio<-1>, ratio<5,3>, ratio<0>, ratio<0>, ratio<1>, 0x7fffffff>&>;
    EXPECT_TRUE(b);
    constexpr bool c = isQuantity<int>;
    EXPECT_FALSE(c); 
}

TEST(Quantity, concept__Isomorphic) {
    constexpr bool a = Isomorphic<Quantity<>, Quantity<>>;
    EXPECT_TRUE(a);

    constexpr bool b = Isomorphic<Quantity<ratio<1>, ratio<-1>, ratio<1,2>, ratio<0>, ratio<0>, ratio<1>>, Quantity<ratio<1>, ratio<-1>, ratio<1,2>, ratio<0>, ratio<0>, ratio<1>>>;
    EXPECT_TRUE(b);

    constexpr bool c = Isomorphic<Quantity<ratio<1>, ratio<0>, ratio<0>, ratio<0>, ratio<0>, ratio<0>, 0>, Quantity<ratio<1>, ratio<0>, ratio<0>, ratio<0>, ratio<0>, ratio<0>, 1>>;
    EXPECT_TRUE(c);

    constexpr bool d = Isomorphic<Quantity<ratio<1>, ratio<1>>, Quantity<>>;
    EXPECT_FALSE(d);

    constexpr bool e = Isomorphic<Quantity<ratio<1>, ratio<1>>, int>;
    EXPECT_FALSE(e);

}

TEST(Quantity, concept__SameFrame_IsomorphicFrame) {
    constexpr bool a = SameFrame<Quantity<>, Quantity<>>;
    EXPECT_TRUE(a);

    constexpr bool b = SameFrame<Quantity<ratio<1>, ratio<0>, ratio<0>, ratio<0>, ratio<0>, ratio<0>, 0>, Quantity<ratio<1>, ratio<0>, ratio<0>, ratio<0>, ratio<0>, ratio<0>, 1>>;
    EXPECT_FALSE(b);

    constexpr bool c = IsomorphicFrame<Quantity<>, Quantity<>>;
    EXPECT_TRUE(c);

    constexpr bool d = IsomorphicFrame<Quantity<ratio<1>, ratio<0>, ratio<0>, ratio<0>, ratio<0>, ratio<0>, 0>, Quantity<ratio<1>, ratio<0>, ratio<0>, ratio<0>, ratio<0>, ratio<0>, 1>>;
    EXPECT_FALSE(d);

    constexpr bool e = IsomorphicFrame<Quantity<>, Quantity<ratio<1>, ratio<0>, ratio<0>, ratio<0>, ratio<0>, ratio<0>, 0>>;
    EXPECT_FALSE(e);
}

TEST(Quantity, named) {
    constexpr bool a = isQuantity<Named<Quantity<>>>;
    EXPECT_TRUE(a);

    constexpr bool b = Isomorphic<Quantity<>, Named<Quantity<>>>;
    EXPECT_TRUE(b);
}