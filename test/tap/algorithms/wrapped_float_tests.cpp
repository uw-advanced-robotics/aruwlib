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
    EXPECT_EQ(5, testInstance.getValue());
}

TEST(WrappedFloat, Wrapping_behavior)
{
    WrappedFloat testInstance(-4, 0, 10);
    EXPECT_EQ(6, testInstance.getValue());

    testInstance.setValue(16);
    EXPECT_EQ(6, testInstance.getValue());

    testInstance.setValue(28);
    EXPECT_EQ(8, testInstance.getValue());
}

TEST(WrappedFloat, Difference)
{
    WrappedFloat testInstance(2, 0, 10);
    EXPECT_EQ(2, testInstance.minDifference(4));
    EXPECT_EQ(-1, testInstance.minDifference(11));

    testInstance.setValue(9);
    EXPECT_EQ(2, testInstance.minDifference(11));

    testInstance.setValue(10);
    EXPECT_EQ(1, testInstance.minDifference(1));
    testInstance.setValue(1);
    EXPECT_EQ(-1, testInstance.minDifference(10));
}

TEST(WrappedFloat, Rotation_bounds)
{
    WrappedFloat testInstance(150, -180, 180);

    EXPECT_EQ(40, testInstance.minDifference(190));
    EXPECT_EQ(40, testInstance.minDifference(-170));

    EXPECT_EQ(40, testInstance.minDifference(190));
    EXPECT_EQ(40, testInstance.minDifference(-170));

    testInstance.setValue(180);

    EXPECT_EQ(180, testInstance.getValue());
    EXPECT_EQ(0, testInstance.minDifference(-180));

    WrappedFloat testInstance2(40, -180, 180);
    EXPECT_EQ(-140, testInstance2.minDifference(-100));
}

TEST(WrappedFloat, Shifting_up)
{
    WrappedFloat testInstance(150, -180, 180);

    testInstance.shiftUp(40);
    EXPECT_EQ(-170, testInstance.getValue());

    testInstance.shiftUp(40);
    EXPECT_EQ(-130, testInstance.getValue());

    testInstance.shiftUp(360);
    EXPECT_EQ(-130, testInstance.getValue());

    testInstance.shiftUp(0);
    EXPECT_EQ(-130, testInstance.getValue());
}

TEST(WrappedFloat, Shifting_down)
{
    WrappedFloat testInstance(150, -180, 180);

    testInstance.shiftDown(40);
    EXPECT_EQ(-170, testInstance.getValue());

    testInstance.shiftDown(40);
    EXPECT_EQ(-130, testInstance.getValue());

    testInstance.shiftDown(360);
    EXPECT_EQ(-130, testInstance.getValue());

    testInstance.shiftDown(0);
    EXPECT_EQ(-130, testInstance.getValue());
}

TEST(WrappedFloat, Bad_bounds)
{
    WrappedFloat testInstance(150, 180, -180);
    EXPECT_EQ(-180, testInstance.getLowerBound());
    EXPECT_EQ(180, testInstance.getUpperBound());
}

TEST(WrappedFloat, shiftBounds_positive)
{
    WrappedFloat testInstance(0, -100, 100);
    EXPECT_EQ(0, testInstance.getValue());
    testInstance.shiftBounds(200);
    EXPECT_EQ(200, testInstance.getValue());
}

TEST(WrappedFloat, shiftBounds_negative)
{
    WrappedFloat testInstance(10, -100, 100);
    EXPECT_EQ(10, testInstance.getValue());
    testInstance.shiftBounds(-200);
    EXPECT_EQ(-190, testInstance.getValue());
}

TEST(WrappedFloat, setLowerBound_value_outside_new_bounds)
{
    WrappedFloat testInstance(10, -100, 100);
    testInstance.setLowerBound(50);
    EXPECT_EQ(60, testInstance.getValue());
}

TEST(WrappedFloat, setUpperBound_value_outside_new_bounds)
{
    WrappedFloat testInstance(10, -100, 100);
    testInstance.setLowerBound(50);
    EXPECT_EQ(60, testInstance.getValue());
}

TEST(WrappedFloat, limitVal_min_lt_max)
{
    WrappedFloat testInstance(0, -100, 100);

    int status;
    EXPECT_EQ(0, WrappedFloat::limitValue(testInstance, -10, 10, &status));
    EXPECT_EQ(0, status);

    testInstance.setValue(-20);
    EXPECT_EQ(-10, WrappedFloat::limitValue(testInstance, -10, 10, &status));
    EXPECT_EQ(1, status);

    testInstance.setValue(20);
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

    testInstance.setValue(20);
    EXPECT_EQ(20, WrappedFloat::limitValue(testInstance, 10, -10, &status));
    EXPECT_EQ(0, status);
}
