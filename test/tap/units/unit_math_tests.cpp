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

#include "tap/units/unit_math.hpp"

using namespace tap::units;
using namespace tap::units::math;

TEST(UnitMath, abs)
{
    Quantity<> q1(-5);
    EXPECT_FLOAT_EQ(5, abs(q1).valueOf());

    Quantity<> q2(0.001);
    EXPECT_FLOAT_EQ(0.001, abs(q2).valueOf());

    Quantity<> q3(0);
    EXPECT_FLOAT_EQ(0, abs(q3).valueOf());
}

TEST(UnitMath, max)
{
    Quantity<> q1(5);
    Quantity<> q2(10);
    EXPECT_FLOAT_EQ(10, max(q1, q2).valueOf());

    Quantity<> q3(-5);
    EXPECT_FLOAT_EQ(5, max(q1, q3).valueOf());

    Quantity<> q4(0);
    EXPECT_FLOAT_EQ(5, max(q1, q4).valueOf());
}

TEST(UnitMath, min)
{
    Quantity<> q1(5);
    Quantity<> q2(10);
    EXPECT_FLOAT_EQ(5, min(q1, q2).valueOf());

    Quantity<> q3(-5);
    EXPECT_FLOAT_EQ(-5, min(q1, q3).valueOf());

    Quantity<> q4(0);
    EXPECT_FLOAT_EQ(0, min(q1, q4).valueOf());
}

TEST(UnitMath, pow)
{
    Quantity<> q1(5);
    EXPECT_FLOAT_EQ(25, square(q1).valueOf());

    Quantity<> q2(2);
    EXPECT_FLOAT_EQ(8, cube(q2).valueOf());

    Quantity<> q3(3);
    EXPECT_FLOAT_EQ(9, pow<2>(q3).valueOf());

    Quantity<> q4(4);
    EXPECT_FLOAT_EQ(64, pow<3>(q4).valueOf());
}

TEST(UnitMath, root)
{
    Quantity<> q1(4);
    EXPECT_FLOAT_EQ(2, sqrt(q1).valueOf());

    Quantity<> q2(27);
    EXPECT_FLOAT_EQ(3, cbrt(q2).valueOf());

    Quantity<> q3(64);
    EXPECT_FLOAT_EQ(4, root<3>(q3).valueOf());

    Quantity<> q4(125);
    EXPECT_FLOAT_EQ(5, root<3>(q4).valueOf());
}

TEST(UnitMath, hypot)
{
    Quantity<> q1(3);
    Quantity<> q2(4);
    EXPECT_NEAR(5, hypot(q1, q2).valueOf(), 1e-5);

    Quantity<> q3(5);
    Quantity<> q4(12);
    EXPECT_NEAR(13, hypot(q3, q4).valueOf(), 1e-5);

    Quantity<> q5(7);
    Quantity<> q6(24);
    EXPECT_NEAR(25, hypot(q5, q6).valueOf(), 1e-5);

    Quantity<> q7(8);
    Quantity<> q8(15);
    EXPECT_NEAR(17, hypot(q7, q8).valueOf(), 1e-5);
}

TEST(UnitMath, mod)
{
    Quantity<> q1(5);
    Quantity<> q2(3);
    EXPECT_FLOAT_EQ(2, mod(q1, q2).valueOf());

    Quantity<> q3(10);
    Quantity<> q4(3);
    EXPECT_FLOAT_EQ(1, mod(q3, q4).valueOf());

    Quantity<> q5(7);
    Quantity<> q6(3);
    EXPECT_FLOAT_EQ(1, mod(q5, q6).valueOf());

    Quantity<> q7(8);
    Quantity<> q8(3);
    EXPECT_FLOAT_EQ(2, mod(q7, q8).valueOf());
}

TEST(UnitMath, copysign)
{
    Quantity<> q1(5);
    Quantity<> q2(-3);
    EXPECT_FLOAT_EQ(-5, copysign(q1, q2).valueOf());

    Quantity<> q3(-5);
    Quantity<> q4(3);
    EXPECT_FLOAT_EQ(5, copysign(q3, q4).valueOf());

    Quantity<> q5(5);
    Quantity<> q6(3);
    EXPECT_FLOAT_EQ(5, copysign(q5, q6).valueOf());

    Quantity<> q7(-5);
    Quantity<> q8(-3);
    EXPECT_FLOAT_EQ(-5, copysign(q7, q8).valueOf());
}

TEST(UnitMath, sgn)
{
    Quantity<> q1(5);
    EXPECT_FLOAT_EQ(1, sgn(q1));

    Quantity<> q2(-5);
    EXPECT_FLOAT_EQ(-1, sgn(q2));

    Quantity<> q3(0);
    EXPECT_FLOAT_EQ(1, sgn(q3));
}

TEST(UnitMath, signbit)
{
    Quantity<> q1(5);
    EXPECT_FALSE(signbit(q1));

    Quantity<> q2(-5);
    EXPECT_TRUE(signbit(q2));

    Quantity<> q3(0);
    EXPECT_FALSE(signbit(q3));
}

TEST(UnitMath, clamp) {
    Quantity<> q1(5);
    Quantity<> q2(0);
    Quantity<> q3(10);
    EXPECT_FLOAT_EQ(5, clamp(q1, q2, q3).valueOf());

    Quantity<> q4(-5);
    Quantity<> q5(0);
    Quantity<> q6(10);
    EXPECT_FLOAT_EQ(0, clamp(q4, q5, q6).valueOf());

    Quantity<> q7(15);
    Quantity<> q8(0);
    Quantity<> q9(10);
    EXPECT_FLOAT_EQ(10, clamp(q7, q8, q9).valueOf());
}

TEST(UnitMath, ceil) {
    Quantity<> q1(5);
    Quantity<> q2(3);
    EXPECT_FLOAT_EQ(6, ceil(q1, q2).valueOf());

    Quantity<> q3(10);
    Quantity<> q4(3);
    EXPECT_FLOAT_EQ(12, ceil(q3, q4).valueOf());

    Quantity<> q5(7);
    Quantity<> q6(3);
    EXPECT_FLOAT_EQ(9, ceil(q5, q6).valueOf());

    Quantity<> q7(8);
    Quantity<> q8(3);
    EXPECT_FLOAT_EQ(9, ceil(q7, q8).valueOf());
}

TEST(UnitMath, floor) {
    Quantity<> q1(5);
    Quantity<> q2(3);
    EXPECT_FLOAT_EQ(3, floor(q1, q2).valueOf());

    Quantity<> q3(10);
    Quantity<> q4(3);
    EXPECT_FLOAT_EQ(9, floor(q3, q4).valueOf());

    Quantity<> q5(7);
    Quantity<> q6(3);
    EXPECT_FLOAT_EQ(6, floor(q5, q6).valueOf());

    Quantity<> q7(8);
    Quantity<> q8(3);
    EXPECT_FLOAT_EQ(6, floor(q7, q8).valueOf());
}

TEST(UnitMath, trunc) {
    Quantity<> q1(5);
    Quantity<> q2(3);
    EXPECT_FLOAT_EQ(3, trunc(q1, q2).valueOf());

    Quantity<> q3(10);
    Quantity<> q4(3);
    EXPECT_FLOAT_EQ(9, trunc(q3, q4).valueOf());

    Quantity<> q5(7);
    Quantity<> q6(3);
    EXPECT_FLOAT_EQ(6, trunc(q5, q6).valueOf());

    Quantity<> q7(8);
    Quantity<> q8(3);
    EXPECT_FLOAT_EQ(6, trunc(q7, q8).valueOf());
}

TEST(UnitMath, round) {
    Quantity<> q1(5);
    Quantity<> q2(3);
    EXPECT_FLOAT_EQ(6, round(q1, q2).valueOf());

    Quantity<> q3(10);
    Quantity<> q4(3);
    EXPECT_FLOAT_EQ(9, round(q3, q4).valueOf());

    Quantity<> q5(7);
    Quantity<> q6(3);
    EXPECT_FLOAT_EQ(6, round(q5, q6).valueOf());

    Quantity<> q7(8);
    Quantity<> q8(3);
    EXPECT_FLOAT_EQ(9, round(q7, q8).valueOf());
}

TEST(UnitMath, wrap) {
    Quantity<> q1(5);
    Quantity<> q2(3);
    Quantity<> q3(7);
    EXPECT_FLOAT_EQ(4, wrap(q1, q2, q3).valueOf());

    Quantity<> q4(10);
    Quantity<> q5(7);
    Quantity<> q6(0);
    EXPECT_FLOAT_EQ(3, wrap(q4, q5, q6).valueOf());

    Quantity<> q7(7);
    Quantity<> q8(3);
    Quantity<> q9(7);
    EXPECT_FLOAT_EQ(6, wrap(q7, q8, q9).valueOf());

    Quantity<> q10(8);
    Quantity<> q11(-7);
    Quantity<> q12(7);
    EXPECT_FLOAT_EQ(1, wrap(q10, q11, q12).valueOf());
}

TEST(UnitMath, sin)
{
    Angle<> a1(0);
    EXPECT_NEAR(0, sin(a1).valueOf(), 1e-5);

    Angle<> a2(M_PI_F / 2);
    EXPECT_NEAR(1, sin(a2).valueOf(), 1e-5);

    Angle<> a3(M_PI_F);
    EXPECT_NEAR(0, sin(a3).valueOf(), 1e-5);

    Angle<> a4(3 * M_PI_F / 2);
    EXPECT_NEAR(-1, sin(a4).valueOf(), 1e-5);
}

TEST(UnitMath, cos)
{
    Angle<> a1(0);
    EXPECT_NEAR(1, cos(a1).valueOf(), 1e-5);

    Angle<> a2(M_PI_F / 2);
    EXPECT_NEAR(0, cos(a2).valueOf(), 1e-5);

    Angle<> a3(M_PI_F);
    EXPECT_NEAR(-1, cos(a3).valueOf(), 1e-5);

    Angle<> a4(3 * M_PI_F / 2);
    EXPECT_NEAR(0, cos(a4).valueOf(), 1e-5);
}

TEST(UnitMath, tan)
{
    Angle<> a1(0);
    EXPECT_NEAR(0, tan(a1).valueOf(), 1e-5);

    Angle<> a2(M_PI_F / 4);
    EXPECT_NEAR(1, tan(a2).valueOf(), 1e-5);

    Angle<> a3(M_PI_F / 2);
    EXPECT_TRUE(abs(tan(a3)).valueOf() > 1'000.0f);

    Angle<> a4(3 * M_PI_F / 4);
    EXPECT_NEAR(-1, tan(a4).valueOf(), 1e-5);
}

TEST(UnitMath, asin)
{
    Quantity<> q1(0);
    EXPECT_NEAR(0, asin(q1).valueOf(), 1e-5);

    Quantity<> q2(1);
    EXPECT_NEAR(M_PI_F / 2, asin(q2).valueOf(), 1e-5);

    Quantity<> q3(-1);
    EXPECT_NEAR(-M_PI_F / 2, asin(q3).valueOf(), 1e-5);

    Quantity<> q4(0.5);
    EXPECT_NEAR(M_PI_F / 6, asin(q4).valueOf(), 1e-5);
}

TEST(UnitMath, acos)
{
    Quantity<> q1(0);
    EXPECT_NEAR(M_PI_F / 2, acos(q1).valueOf(), 1e-5);

    Quantity<> q2(1);
    EXPECT_NEAR(0, acos(q2).valueOf(), 1e-5);

    Quantity<> q3(-1);
    EXPECT_NEAR(M_PI_F, acos(q3).valueOf(), 1e-5);

    Quantity<> q4(0.5);
    EXPECT_NEAR(M_PI_F / 3, acos(q4).valueOf(), 1e-5);
}

TEST(UnitMath, atan)
{
    Quantity<> q1(0);
    EXPECT_NEAR(0, atan(q1).valueOf(), 1e-5);

    Quantity<> q2(1);
    EXPECT_NEAR(M_PI_F / 4, atan(q2).valueOf(), 1e-5);

    Quantity<> q3(-1);
    EXPECT_NEAR(-M_PI_F / 4, atan(q3).valueOf(), 1e-5);

    Quantity<> q4(0);
    Quantity<> q5(1);
    EXPECT_NEAR(0, atan2(q4, q5).valueOf(), 1e-5);

    Quantity<> q6(1);
    Quantity<> q7(1);
    EXPECT_NEAR(M_PI_F / 4, atan2(q6, q7).valueOf(), 1e-5);
}

TEST(UnitMath, compareClose) {
    Quantity<> q1(5);
    Quantity<> q2(5.0001);
    EXPECT_TRUE(compareClose(q1, q2, Quantity<>(0.001)));

    Quantity<> q3(5);
    Quantity<> q4(5.002);
    EXPECT_FALSE(compareClose(q3, q4, Quantity<>(0.001)));
}
