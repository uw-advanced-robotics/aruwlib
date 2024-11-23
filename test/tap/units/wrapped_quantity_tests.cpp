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

TEST(WrappedQuantity, Basic_functionality)
{
    static_assert(
        isWrappedQuantity<Wrapped<Number<>>>,
        "Wrapped<Number<>> is not a Wrapped Quantity");
    static_assert(isQuantity<Wrapped<Number<>>>, "Wrapped<Number<>> is not a Quantity");
    static_assert(isQuantity<Wrapped<int>>, "Wrapped<int> is not a Quantity");
    Wrapped<Number<>> testInstance(5_n, 0_n, 10_n);
    EXPECT_EQ(5, testInstance.valueOf());
    EXPECT_EQ(5, testInstance.valueOfUnwrapped());
}

TEST(WrappedQuantity, Wrapping_behavior)
{
    Wrapped<Number<>> testInstance(-4_n, 0_n, 10_n);

    EXPECT_EQ(6, testInstance.valueOf());
    EXPECT_EQ(-4, testInstance.valueOfUnwrapped());

    testInstance.set(16_n);
    EXPECT_EQ(6, testInstance.valueOf());
    EXPECT_EQ(6, testInstance.valueOfUnwrapped());

    testInstance.set(28_n);
    EXPECT_EQ(8, testInstance.valueOf());
    EXPECT_EQ(28, testInstance.valueOfUnwrapped());

    Wrapped<int> testInstance2(-4, 0, 10);
    EXPECT_EQ(6, testInstance2.valueOf());
    EXPECT_EQ(-4, testInstance2.valueOfUnwrapped());

    testInstance2.set(16);
    EXPECT_EQ(6, testInstance2.valueOf());
    EXPECT_EQ(6, testInstance2.valueOfUnwrapped());

    testInstance2.set(28);
    EXPECT_EQ(8, testInstance2.valueOf());
    EXPECT_EQ(28, testInstance2.valueOfUnwrapped());
}

TEST(WrappedQuantity, Difference)
{
    Wrapped<Number<>> testInstance(2_n, 0_n, 10_n);
    EXPECT_EQ(2_n, testInstance.minDifference(4_n));
    EXPECT_EQ(-1_n, testInstance.minDifference(11_n));

    testInstance.set(9_n);
    EXPECT_EQ(2_n, testInstance.minDifference(11_n));

    testInstance.set(10_n);
    EXPECT_EQ(1_n, testInstance.minDifference(1_n));
    testInstance.set(1_n);
    EXPECT_EQ(-1_n, testInstance.minDifference(10_n));

    Wrapped<float> testInstance2(2, 0, 10);
    EXPECT_EQ(2, testInstance2.minDifference(4));
    EXPECT_EQ(-1, testInstance2.minDifference(11));
}

TEST(WrappedQuantity, Rotation_bounds)
{
    Wrapped<Angle<>> testInstance(150_deg, -180_deg, 180_deg);

    EXPECT_NEAR(40, toDegree(testInstance.minDifference(190_deg)), 1E-3);
    EXPECT_NEAR(40, toDegree(testInstance.minDifference(-170_deg)), 1E-3);

    EXPECT_NEAR(40, toDegree(testInstance.minDifference(190_deg)), 1E-3);
    EXPECT_NEAR(40, toDegree(testInstance.minDifference(-170_deg)), 1E-3);

    testInstance.set(180_deg);

    EXPECT_NEAR(-180, toDegree(testInstance), 1E-3);
    EXPECT_NEAR(0, toDegree(testInstance.minDifference(-180_deg)), 1E-3);

    Wrapped<Angle<>> testInstance2(40_deg, -180_deg, 180_deg);
    EXPECT_NEAR(-140, toDegree(testInstance2.minDifference(-100_deg)), 1E-3);
}

TEST(WrappedQuantity, Shifting_up)
{
    Wrapped<Number<>> testInstance(150_n, -180_n, 180_n);

    testInstance += 40_n;
    EXPECT_EQ(-170, testInstance.valueOf());
    EXPECT_EQ(190, testInstance.valueOfUnwrapped());

    testInstance += 40_n;
    EXPECT_EQ(-130, testInstance.valueOf());
    EXPECT_EQ(230, testInstance.valueOfUnwrapped());

    testInstance += 360_n;
    EXPECT_EQ(-130, testInstance.valueOf());
    EXPECT_EQ(590, testInstance.valueOfUnwrapped());

    testInstance += 0_n;
    EXPECT_EQ(-130, testInstance.valueOf());
    EXPECT_EQ(590, testInstance.valueOfUnwrapped());

    Wrapped<char> testInstance2(100, 20, 120);
    testInstance2 += (char)20;
    EXPECT_EQ(20, testInstance2.valueOf());
}

TEST(WrappedQuantity, shifting_down)
{
    Wrapped<Number<>> testInstance(-150_n, -180_n, 180_n);

    testInstance -= 40_n;
    EXPECT_EQ(170, testInstance.valueOf());
    EXPECT_EQ(-190, testInstance.valueOfUnwrapped());

    testInstance -= 40_n;
    EXPECT_EQ(130, testInstance.valueOf());
    EXPECT_EQ(-230, testInstance.valueOfUnwrapped());

    testInstance -= 360_n;
    EXPECT_EQ(130, testInstance.valueOf());
    EXPECT_EQ(-590, testInstance.valueOfUnwrapped());

    testInstance -= 0_n;
    EXPECT_EQ(130, testInstance.valueOf());
    EXPECT_EQ(-590, testInstance.valueOfUnwrapped());

    Wrapped<char> testInstance2(20, 20, 120);
    testInstance2 -= (char)20;
    EXPECT_EQ(100, testInstance2.valueOf());
}

TEST(WrappedQuantity, shiftBounds_positive)
{
    Wrapped<Number<>> testInstance(0_n, -100_n, 100_n);
    EXPECT_EQ(0, testInstance.valueOf());
    testInstance.shiftBounds(200_n);
    EXPECT_EQ(200, testInstance.valueOf());
}

TEST(WrappedQuantity, shiftBounds_negative)
{
    Wrapped<float> testInstance(10, -100, 100);
    EXPECT_EQ(10, testInstance.valueOf());
    testInstance.shiftBounds(-200);
    EXPECT_EQ(-190, testInstance.valueOf());
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

TEST(WrappedQuantity, operator__add_subtract)
{
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
    Wrapped<int> q4(5, 0, 10);
    Wrapped<int> q5 = q4 + 3;
    EXPECT_EQ(8, q5.valueOf());

    q5 = q4 - 3;
    EXPECT_EQ(2, q5.valueOf());
}

TEST(WrappedQuantity, operator__scalar_multiply_divide)
{
    Wrapped<Length<>> q1(5_m, 0_m, 10_m);
    auto q2 = q1 * 2;
    EXPECT_FLOAT_EQ(0, q2.valueOf());
    EXPECT_FLOAT_EQ(0, q2.getLowerBound().valueOf());
    EXPECT_FLOAT_EQ(10, q2.getUpperBound().valueOf());

    q2 = q1 / 2;
    EXPECT_FLOAT_EQ(2.5, q2.valueOf());
    EXPECT_FLOAT_EQ(0, q2.getLowerBound().valueOf());
    EXPECT_FLOAT_EQ(10, q2.getUpperBound().valueOf());

    Wrapped<int> q3(102, 0, 10);
    auto q4 = q3 * 2;
    EXPECT_EQ(4, q4.valueOf());
    EXPECT_EQ(204, q4.valueOfUnwrapped());
}
