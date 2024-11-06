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

#include "tap/units/units.hpp"
#include "tap/units/wrapped_quantity.hpp"
using namespace tap::units;
using namespace tap::units::conversions;
using namespace tap::units::literals;

TEST(WrappedQuantity, constructors__bounds_revolutions)
{
    Wrapped<Number<>> q1(5_n, 0_n, 10_n);
    EXPECT_FLOAT_EQ(5, q1.valueOf());
    EXPECT_FLOAT_EQ(0, q1.getLowerBound().valueOf());
    EXPECT_FLOAT_EQ(10, q1.getUpperBound().valueOf());
    EXPECT_EQ(0, q1.getRevolutions());

    Wrapped<Length<>> q2(11_m, 0_m, 10_m);
    EXPECT_FLOAT_EQ(1, q2.valueOf());
    EXPECT_EQ(1, q2.getRevolutions());

    Wrapped<Length<>> q3(11_m, 0_m, 5_m);
    EXPECT_FLOAT_EQ(1, q3.valueOf());
    EXPECT_EQ(2, q3.getRevolutions());
}

TEST(WrappedQuantity, unwrap)
{
    Wrapped<Number<>> q1(5_n, 0_n, 10_n);
    EXPECT_FLOAT_EQ(5, q1.valueOf());
    EXPECT_FLOAT_EQ(5, q1.getUnwrapped().valueOf());

    Wrapped<Length<>> q2(11_m, 0_m, 10_m);
    EXPECT_FLOAT_EQ(1, q2.valueOf());
    EXPECT_FLOAT_EQ(11, q2.getUnwrapped().valueOf());
}

TEST(WrappedQuantity, withSameBounds)
{
    Wrapped<Length<>> q1(5_m, 0_m, 10_m);
    auto q2 = q1.withSameBounds(6_m);
    EXPECT_FLOAT_EQ(6, q2.valueOf());
    EXPECT_FLOAT_EQ(0, q2.getLowerBound().valueOf());
    EXPECT_FLOAT_EQ(10, q2.getUpperBound().valueOf());

    auto q3 = q1.withSameBounds(10_m, 0.5, 2);
    EXPECT_FLOAT_EQ(10, q3.valueOf());
    EXPECT_FLOAT_EQ(0, q3.getLowerBound().valueOf());
    EXPECT_FLOAT_EQ(20, q3.getUpperBound().valueOf());

    auto q4 = q1.withSameBounds(12_m, 0.5, 0.5);
    EXPECT_FLOAT_EQ(2, q4.valueOf());
    EXPECT_FLOAT_EQ(0, q4.getLowerBound().valueOf());
    EXPECT_FLOAT_EQ(5, q4.getUpperBound().valueOf());
    EXPECT_EQ(2, q4.getRevolutions());
}

TEST(WrappedQuantity, withinRange)
{
    Wrapped<Length<>> q1(5_m, 0_m, 10_m);
    EXPECT_TRUE(q1.withinRange(5_m));
    EXPECT_TRUE(q1.withinRange(0_m));
    EXPECT_FALSE(q1.withinRange(10_m));
    EXPECT_FALSE(q1.withinRange(11_m));
    EXPECT_FALSE(q1.withinRange(fromMeter(INFINITY)));
}

TEST(WrappedQuantity, operator__add_subtract_equals)
{
    Wrapped<Length<>> q1(5_m, 0_m, 10_m);
    q1 += 5_m;
    EXPECT_FLOAT_EQ(0, q1.valueOf());
    EXPECT_FLOAT_EQ(0, q1.getLowerBound().valueOf());
    EXPECT_FLOAT_EQ(10, q1.getUpperBound().valueOf());

    q1 -= 6_m;
    EXPECT_FLOAT_EQ(4, q1.valueOf());
    EXPECT_FLOAT_EQ(0, q1.getLowerBound().valueOf());
    EXPECT_FLOAT_EQ(10, q1.getUpperBound().valueOf());
}

TEST(WrappedQuantity, operator__multiply_divide_equals)
{
    Wrapped<Length<>> q1(5_m, 0_m, 10_m);
    q1 *= 2;
    EXPECT_FLOAT_EQ(0, q1.valueOf());
    EXPECT_FLOAT_EQ(0, q1.getLowerBound().valueOf());
    EXPECT_FLOAT_EQ(10, q1.getUpperBound().valueOf());

    q1 += 5_m;
    q1 /= 2;
    EXPECT_FLOAT_EQ(2.5, q1.valueOf());
}

TEST(WrappedQuantity, minDifference) {
    Wrapped<Length<>> q1(5_m, 0_m, 10_m);
    Wrapped<Length<>> q2(6_m, 0_m, 10_m);
    EXPECT_FLOAT_EQ(1, q1.minDifference(q2).valueOf());
    EXPECT_FLOAT_EQ(-1, q2.minDifference(q1).valueOf());
}

TEST(WrappedQuantity, shiftBounds)
{
    Wrapped<Length<>> q1(5_m, 0_m, 10_m);
    q1.shiftBounds(5_m);
    EXPECT_FLOAT_EQ(5, q1.valueOf());
    EXPECT_FLOAT_EQ(5, q1.getLowerBound().valueOf());
    EXPECT_FLOAT_EQ(15, q1.getUpperBound().valueOf());

    q1.shiftBounds(fromMeter(-10));
    EXPECT_FLOAT_EQ(-5, q1.valueOf());
    EXPECT_FLOAT_EQ(-5, q1.getLowerBound().valueOf());
    EXPECT_FLOAT_EQ(5, q1.getUpperBound().valueOf());
}

TEST(WrappedQuantity, operator__add_subtract) {
    Wrapped<Length<>> q1(5_m, 0_m, 10_m);
    Wrapped<Length<>> q2(6_m, 0_m, 10_m);
    auto q3 = q1 + q2;
    EXPECT_FLOAT_EQ(1, q3.valueOf());
    EXPECT_FLOAT_EQ(0, q3.getLowerBound().valueOf());
    EXPECT_FLOAT_EQ(10, q3.getUpperBound().valueOf());

    q3 = q1 - q2;
    EXPECT_FLOAT_EQ(9, q3.valueOf());
    EXPECT_FLOAT_EQ(0, q3.getLowerBound().valueOf());
    EXPECT_FLOAT_EQ(10, q3.getUpperBound().valueOf());
}

TEST(WrappedQuantity, operator__scalar_multiply_divide) {
    Wrapped<Length<>> q1(5_m, 0_m, 10_m);
    auto q2 = q1 * 2;
    EXPECT_FLOAT_EQ(0, q2.valueOf());
    EXPECT_FLOAT_EQ(0, q2.getLowerBound().valueOf());
    EXPECT_FLOAT_EQ(10, q2.getUpperBound().valueOf());

    q2 = q1 / 2;
    EXPECT_FLOAT_EQ(2.5, q2.valueOf());
    EXPECT_FLOAT_EQ(0, q2.getLowerBound().valueOf());
    EXPECT_FLOAT_EQ(10, q2.getUpperBound().valueOf());
}

