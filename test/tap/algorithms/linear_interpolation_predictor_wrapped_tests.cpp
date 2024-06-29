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

#include "tap/algorithms/linear_interpolation_predictor_wrapped.hpp"

using namespace tap::algorithms;

TEST(
    LinearInterpolationPredictorWrapped,
    update_return_same_value_as_reset_if_time_before_reset_time_used)
{
    LinearInterpolationPredictorWrapped li(0, 10);

    li.reset(5, 10);
    li.update(6, 5);
    EXPECT_NEAR(5, li.getInterpolatedValue(100), 1E-3);
    li.update(3, 1);
    EXPECT_NEAR(5, li.getInterpolatedValue(100), 1E-3);
}

TEST(LinearInterpolationPredictorWrapped, after_reset_return_reset_initial_val)
{
    LinearInterpolationPredictorWrapped li(0, 10);

    li.reset(3, 5);
    EXPECT_NEAR(3, li.getInterpolatedValue(100), 1E-3);
    li.reset(7, 1);
    EXPECT_NEAR(7, li.getInterpolatedValue(100), 1E-3);
}

TEST(LinearInterpolationPredictorWrapped, normal_operation_no_wrapping)
{
    LinearInterpolationPredictorWrapped li(0, 10);

    li.reset(3, 10);
    EXPECT_NEAR(3, li.getInterpolatedValue(15), 1E-3);
    li.update(4, 20);
    EXPECT_NEAR(4.5, li.getInterpolatedValue(25), 1E-3);
    li.update(6, 25);
    EXPECT_NEAR(6.4, li.getInterpolatedValue(26), 1E-3);
    li.update(4, 26);
    EXPECT_NEAR(2, li.getInterpolatedValue(27), 1E-3);
}

TEST(LinearInterpolationPredictorWrapped, normal_operation_wrapping)
{
    LinearInterpolationPredictorWrapped li(0, 10);

    li.reset(1, 10);
    EXPECT_NEAR(1, li.getInterpolatedValue(11), 1E-3);
    li.update(9, 12);
    EXPECT_NEAR(7, li.getInterpolatedValue(14), 1E-3);

    li.reset(1, 10);
    li.update(0.5, 11);
    EXPECT_NEAR(9.5, li.getInterpolatedValue(13), 1E-3);

    li.reset(9, 10);
    li.update(1, 11);
    EXPECT_NEAR(3, li.getInterpolatedValue(12), 1E-3);

    li.reset(9, 10);
    li.update(9.5, 11);
    EXPECT_NEAR(0.5, li.getInterpolatedValue(13), 1E-3);
}

TEST(LinearInterpolationPredictorWrapped, after_update_with_bad_time_value_doesnt_change)
{
    LinearInterpolationPredictorWrapped li(0, 10);

    li.reset(5, 1);
    li.update(6, 2);
    EXPECT_NEAR(7, li.getInterpolatedValue(3), 1E-3);
    li.update(7, 1);
    // Time before last update time, slope should be 0, getInterpolatedValue returns last value
    EXPECT_NEAR(6, li.getInterpolatedValue(4), 1E-3);
}
