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

#include "tap/communication/sensors/imu_heater/imu_heater.hpp"
#include "tap/communication/sensors/imu_heater/imu_heater_constants.hpp"
#include "tap/drivers.hpp"

using namespace tap::communication::sensors::imu_heater;
using namespace testing;

class ImuHeaterTest : public Test
{
protected:
    ImuHeaterTest() : heater(&drivers) {}

    void SetUp() override
    {
        ON_CALL(drivers.pwm, write)
            .WillByDefault([&](float duty, tap::gpio::Pwm::Pin) { imuHeaterOutput = duty; });
    }

    tap::Drivers drivers;
    ImuHeater heater;
    float imuHeaterOutput = 0;

    static constexpr float IMU_DESIRED_TEMPERATURE = 50;
};

TEST_F(ImuHeaterTest, initialize_sets_pwm_freq)
{
    EXPECT_CALL(drivers.pwm, setTimerFrequency(bound_ports::IMU_HEATER_TIMER, _));

    heater.initialize();
}

TEST_F(ImuHeaterTest, runTemperatureController_negative_temperature_writes_0_duty_cycle)
{
    heater.runTemperatureController(-1);

    EXPECT_NEAR(0, imuHeaterOutput, 1E-3);
}

TEST_F(ImuHeaterTest, runTemperatureController_temp_gt_IMU_DESIRED_TEMPERATURE_writes_0_duty_cycle)
{
    heater.runTemperatureController(IMU_DESIRED_TEMPERATURE + 10);

    EXPECT_NEAR(0, imuHeaterOutput, 1E-3);
}

TEST_F(ImuHeaterTest, runTemperatureController_temp_lt_IMU_DESIRED_TEMPERATURE_writes_1_duty_cycle)
{
    heater.runTemperatureController(std::max(0.0f, IMU_DESIRED_TEMPERATURE - 10.0f));

    EXPECT_NEAR(1, imuHeaterOutput, 1E-3);
}

MATCHER_P2(
    IsBetween,
    a,
    b,
    std::string(negation ? "isn't" : "is") + " between " + PrintToString(a) + " and " +
        PrintToString(b))
{
    return a <= arg && arg <= b;
}

TEST_F(ImuHeaterTest, runTemperatureController_output_always_between_0_and_1)
{
    std::vector<float> temperatures{-10, 0, 10, 20, 10000};
    for (float t : temperatures)
    {
        // call multiple times to allow PID to react
        for (int i = 0; i < 10; i++)
        {
            heater.runTemperatureController(t);
            EXPECT_THAT(imuHeaterOutput, IsBetween(0, 1));
        }
    }
}

TEST_F(ImuHeaterTest, runTemperatureController_output_changes_with_temperature_change)
{
    int changedTemperature = IMU_DESIRED_TEMPERATURE - 15;

    heater.runTemperatureController(changedTemperature);

    EXPECT_NEAR(1, imuHeaterOutput, 1E-3);

    heater.setDesiredTemperature(changedTemperature);

    heater.runTemperatureController(changedTemperature);

    EXPECT_NEAR(0, imuHeaterOutput, 1E-3);
}
