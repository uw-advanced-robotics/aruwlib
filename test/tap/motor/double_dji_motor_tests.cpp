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

#include "tap/drivers.hpp"
#include "tap/motor/double_dji_motor.hpp"

using namespace testing;
using namespace tap::motor;

#define SETUP_TEST()                \
    tap::Drivers drivers;           \
    DoubleDjiMotor motor(           \
        &drivers,                   \
        MotorId::MOTOR1,            \
        MotorId::MOTOR2,            \
        tap::can::CanBus::CAN_BUS1, \
        tap::can::CanBus::CAN_BUS1, \
        false,                      \
        false,                      \
        "m1",                       \
        "m2");

static constexpr uint16_t ENC_RESOLUTION = 8192;

TEST(DoubleDjiMotor, initialize__both_motors_initialized)
{
    SETUP_TEST();

    EXPECT_CALL(motor.motorOne, initialize);
    EXPECT_CALL(motor.motorTwo, initialize);

    motor.initialize();
}

TEST(DoubleDjiMotor, setDesiredOutput__sets_both_motors_output)
{
    SETUP_TEST();

    int32_t expectedDesOut = 0;

    EXPECT_CALL(motor.motorOne, setDesiredOutput).WillRepeatedly([&](int32_t desOut) {
        EXPECT_EQ(expectedDesOut, desOut);
    });
    EXPECT_CALL(motor.motorTwo, setDesiredOutput).WillRepeatedly([&](int32_t desOut) {
        EXPECT_EQ(expectedDesOut, desOut);
    });

    std::vector<int32_t> possibleDesOut{-30000, -15000, -1000, 0, 1500, 12434};
    for (int32_t desOut : possibleDesOut)
    {
        expectedDesOut = desOut;
        motor.setDesiredOutput(expectedDesOut);
    }
}

TEST(DoubleDjiMotor, isMotorOnline__returns_true_if_both_motors_online_otherwise_false)
{
    SETUP_TEST();

    ON_CALL(motor.motorOne, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(motor.motorTwo, isMotorOnline).WillByDefault(Return(false));
    EXPECT_FALSE(motor.isMotorOnline());

    ON_CALL(motor.motorOne, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(motor.motorTwo, isMotorOnline).WillByDefault(Return(false));
    EXPECT_FALSE(motor.isMotorOnline());

    ON_CALL(motor.motorOne, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(motor.motorTwo, isMotorOnline).WillByDefault(Return(true));
    EXPECT_FALSE(motor.isMotorOnline());

    ON_CALL(motor.motorOne, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(motor.motorTwo, isMotorOnline).WillByDefault(Return(true));
    EXPECT_TRUE(motor.isMotorOnline());
}

TEST(DoubleDjiMotor, getOutputDesired__returns_average_output_desired)
{
    SETUP_TEST();

    int16_t motorOneDesOut = 0, motorTwoDesOut = 0;

    ON_CALL(motor.motorOne, getOutputDesired).WillByDefault(ReturnPointee(&motorOneDesOut));
    ON_CALL(motor.motorTwo, getOutputDesired).WillByDefault(ReturnPointee(&motorTwoDesOut));

    EXPECT_EQ(0, motor.getOutputDesired());

    motorOneDesOut = -1000;
    motorTwoDesOut = -1000;
    EXPECT_EQ(-1000, motor.getOutputDesired());

    motorOneDesOut = -1000;
    motorTwoDesOut = 1000;
    EXPECT_EQ(0, motor.getOutputDesired());

    motorOneDesOut = 750;
    motorTwoDesOut = 1250;
    EXPECT_EQ(1000, motor.getOutputDesired());
}

TEST(DoubleDjiMotor, getTemperature__returns_average_temperature)
{
    SETUP_TEST();

    int8_t motorOneTemp = 0, motorTwoTemp = 0;

    ON_CALL(motor.motorOne, getTemperature).WillByDefault(ReturnPointee(&motorOneTemp));
    ON_CALL(motor.motorTwo, getTemperature).WillByDefault(ReturnPointee(&motorTwoTemp));

    EXPECT_EQ(0, motor.getTemperature());

    motorOneTemp = -100;
    motorTwoTemp = -100;
    EXPECT_EQ(-100, motor.getTemperature());

    motorOneTemp = -100;
    motorTwoTemp = 100;
    EXPECT_EQ(100, motor.getTemperature());

    motorOneTemp = 120;
    motorTwoTemp = 120;
    EXPECT_EQ(120, motor.getTemperature());
}

TEST(DoubleDjiMotor, getTorque__returns_average_torque)
{
    SETUP_TEST();

    int16_t motorOneTorque = 0, motorTwoTorque = 0;

    ON_CALL(motor.motorOne, getTorque).WillByDefault(ReturnPointee(&motorOneTorque));
    ON_CALL(motor.motorTwo, getTorque).WillByDefault(ReturnPointee(&motorTwoTorque));

    EXPECT_EQ(0, motor.getTorque());

    motorOneTorque = -1000;
    motorTwoTorque = -1000;
    EXPECT_EQ(-1000, motor.getTorque());

    motorOneTorque = -1000;
    motorTwoTorque = 1000;
    EXPECT_EQ(0, motor.getTorque());

    motorOneTorque = 2000;
    motorTwoTorque = 2000;
    EXPECT_EQ(2000, motor.getTorque());
}
