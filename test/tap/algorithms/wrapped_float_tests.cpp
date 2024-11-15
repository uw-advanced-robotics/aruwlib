/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/algorithms/wrapped_float.hpp"

using namespace tap::algorithms;

TEST(WrappedFloat, Basic_functionality)
{
    WrappedFloat testInstance(5, 0, 10);
    EXPECT_EQ(5, testInstance.getWrappedValue());
}

TEST(WrappedFloat, Wrapping_behavior)
{
    WrappedFloat testInstance(-4, 0, 10);
    EXPECT_EQ(6, testInstance.getWrappedValue());
    EXPECT_EQ(-4, testInstance.getUnwrappedValue());

    testInstance.setWrappedValue(16);
    EXPECT_EQ(6, testInstance.getWrappedValue());
    EXPECT_EQ(6, testInstance.getUnwrappedValue());

    testInstance.setWrappedValue(28);
    EXPECT_EQ(8, testInstance.getWrappedValue());
    EXPECT_EQ(28, testInstance.getUnwrappedValue());
}

TEST(WrappedFloat, Difference)
{
    WrappedFloat testInstance(2, 0, 10);
    EXPECT_EQ(2, testInstance.minDifference(4));
    EXPECT_EQ(-1, testInstance.minDifference(11));

    testInstance.setWrappedValue(9);
    EXPECT_EQ(2, testInstance.minDifference(11));

    testInstance.setWrappedValue(10);
    EXPECT_EQ(1, testInstance.minDifference(1));
    testInstance.setWrappedValue(1);
    EXPECT_EQ(-1, testInstance.minDifference(10));
}

TEST(WrappedFloat, Rotation_bounds)
{
    WrappedFloat testInstance(150, -180, 180);

    EXPECT_EQ(40, testInstance.minDifference(190));
    EXPECT_EQ(40, testInstance.minDifference(-170));

    EXPECT_EQ(40, testInstance.minDifference(190));
    EXPECT_EQ(40, testInstance.minDifference(-170));

    testInstance.setWrappedValue(180);

    EXPECT_EQ(-180, testInstance.getWrappedValue());
    EXPECT_EQ(0, testInstance.minDifference(-180));

    WrappedFloat testInstance2(40, -180, 180);
    EXPECT_EQ(-140, testInstance2.minDifference(-100));
}

TEST(WrappedFloat, Shifting_up)
{
    WrappedFloat testInstance(150, -180, 180);

    testInstance += 40;
    EXPECT_EQ(-170, testInstance.getWrappedValue());
    EXPECT_EQ(190, testInstance.getUnwrappedValue());

    testInstance += 40;
    EXPECT_EQ(-130, testInstance.getWrappedValue());
    EXPECT_EQ(230, testInstance.getUnwrappedValue());

    testInstance += 360;
    EXPECT_EQ(-130, testInstance.getWrappedValue());
    EXPECT_EQ(590, testInstance.getUnwrappedValue());

    testInstance += 0;
    EXPECT_EQ(-130, testInstance.getWrappedValue());
    EXPECT_EQ(590, testInstance.getUnwrappedValue());
}

TEST(WrappedFloat, shifting_down)
{
    WrappedFloat testInstance(-150, -180, 180);

    testInstance -= 40;
    EXPECT_EQ(170, testInstance.getWrappedValue());
    EXPECT_EQ(-190, testInstance.getUnwrappedValue());

    testInstance -= 40;
    EXPECT_EQ(130, testInstance.getWrappedValue());
    EXPECT_EQ(-230, testInstance.getUnwrappedValue());

    testInstance -= 360;
    EXPECT_EQ(130, testInstance.getWrappedValue());
    EXPECT_EQ(-590, testInstance.getUnwrappedValue());

    testInstance -= 0;
    EXPECT_EQ(130, testInstance.getWrappedValue());
    EXPECT_EQ(-590, testInstance.getUnwrappedValue());
}

TEST(WrappedFloat, shiftBounds_positive)
{
    WrappedFloat testInstance(0, -100, 100);
    EXPECT_EQ(0, testInstance.getWrappedValue());
    testInstance.shiftBounds(200);
    EXPECT_EQ(200, testInstance.getWrappedValue());
}

TEST(WrappedFloat, shiftBounds_negative)
{
    WrappedFloat testInstance(10, -100, 100);
    EXPECT_EQ(10, testInstance.getWrappedValue());
    testInstance.shiftBounds(-200);
    EXPECT_EQ(-190, testInstance.getWrappedValue());
}

TEST(WrappedFloat, limitVal_min_lt_max)
{
    WrappedFloat testInstance(0, -100, 100);

    int status;
    EXPECT_EQ(0, WrappedFloat::limitValue(testInstance, -10, 10, &status));
    EXPECT_EQ(0, status);

    testInstance.setWrappedValue(-20);
    EXPECT_EQ(-10, WrappedFloat::limitValue(testInstance, -10, 10, &status));
    EXPECT_EQ(1, status);

    testInstance.setWrappedValue(20);
    EXPECT_EQ(10, WrappedFloat::limitValue(testInstance, -10, 10, &status));
    EXPECT_EQ(2, status);
}

TEST(WrappedFloat, limitVal_min_gt_max)
{
    WrappedFloat testInstance(0, -100, 100);

    int status;

    EXPECT_EQ(5, WrappedFloat::limitValue(testInstance, 5, -10, &status));
    EXPECT_EQ(1, status);

    EXPECT_EQ(-5, WrappedFloat::limitValue(testInstance, 10, -5, &status));
    EXPECT_EQ(2, status);

    testInstance.setWrappedValue(20);
    EXPECT_EQ(20, WrappedFloat::limitValue(testInstance, 10, -10, &status));
    EXPECT_EQ(0, status);
}

TEST(WrappedFloat, rangeOverlap)
{
    // non intersecting ranges
    EXPECT_EQ(
        0.0f,
        WrappedFloat::rangeOverlap(
            WrappedFloat(0, 0, 100),
            WrappedFloat(49, 0, 100),
            WrappedFloat(50, 0, 100),
            WrappedFloat(90, 0, 100)));

    // basic intersection
    EXPECT_EQ(
        20.0f,
        WrappedFloat::rangeOverlap(
            WrappedFloat(0, 0, 100),
            WrappedFloat(60, 0, 100),
            WrappedFloat(40, 0, 100),
            WrappedFloat(90, 0, 100)));

    EXPECT_EQ(
        20.0f,
        WrappedFloat::rangeOverlap(
            WrappedFloat(40, 0, 100),
            WrappedFloat(90, 0, 100),
            WrappedFloat(0, 0, 100),
            WrappedFloat(60, 0, 100)));

    // one range contained entirely in the other
    EXPECT_EQ(
        30.0f,
        WrappedFloat::rangeOverlap(
            WrappedFloat(0, 0, 100),
            WrappedFloat(90, 0, 100),
            WrappedFloat(30, 0, 100),
            WrappedFloat(60, 0, 100)));

    EXPECT_EQ(
        30.0f,
        WrappedFloat::rangeOverlap(
            WrappedFloat(30, 0, 100),
            WrappedFloat(60, 0, 100),
            WrappedFloat(0, 0, 100),
            WrappedFloat(90, 0, 100)));

    // two intersections
    EXPECT_EQ(
        20.0f,
        WrappedFloat::rangeOverlap(
            WrappedFloat(30, 0, 100),
            WrappedFloat(60, 0, 100),
            WrappedFloat(50, 0, 100),
            WrappedFloat(40, 0, 100)));

    // one adjacent boundary, no intersection
    EXPECT_EQ(
        0.0f,
        WrappedFloat::rangeOverlap(
            WrappedFloat(30, 0, 100),
            WrappedFloat(60, 0, 100),
            WrappedFloat(60, 0, 100),
            WrappedFloat(90, 0, 100)));

    // one adjacent boundary, one intersection
    EXPECT_EQ(
        20.0f,
        WrappedFloat::rangeOverlap(
            WrappedFloat(30, 0, 100),
            WrappedFloat(80, 0, 100),
            WrappedFloat(60, 0, 100),
            WrappedFloat(30, 0, 100)));

    // duplicate range
    EXPECT_EQ(
        30.0f,
        WrappedFloat::rangeOverlap(
            WrappedFloat(30, 0, 100),
            WrappedFloat(60, 0, 100),
            WrappedFloat(30, 0, 100),
            WrappedFloat(60, 0, 100)));
}
