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

using namespace tap::units;
using namespace tap::units::constants;
using namespace tap::units::conversions;
using namespace tap::units::literals;

TEST(Units, classes__equivalent) {}

TEST(Units, literals)
{
    EXPECT_FLOAT_EQ(1, (1_m).valueOf());
    EXPECT_NEAR(0.001, (1_ms).valueOf(), 1e-3);
    EXPECT_NEAR(0.45359237, (1_lb).valueOf(), 1e-3);
}

TEST(Units, temperature)
{
    Temperature<> t1(0);
    EXPECT_FLOAT_EQ(0, t1.valueOf());
    EXPECT_NEAR(-273.15, toCelsius(t1), 1e-5);

    Temperature<> t2 = 0_degC;
    EXPECT_NEAR(273.15, t2.valueOf(), 1e-5);
    EXPECT_NEAR(32, toFahrenheit(t2), 1e-5);

    Temperature<> t3 = fromCelsius(-40);
    EXPECT_NEAR(233.15, t3.valueOf(), 1e-5);
    EXPECT_NEAR(-40, toFahrenheit(t3), 1e-5);
}