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

#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

using namespace tap::motor;

TEST(DjiMotor, getName_returns_name)
{
    tap::Drivers drivers;
    DjiMotor motor(&drivers, MOTOR1, tap::can::CanBus::CAN_BUS1, false, "cool motor");

    EXPECT_EQ(std::string("cool motor"), std::string(motor.getName()));
}

TEST(DjiMotor, initiailze_adds_self_to_motor_handlers)
{
    tap::Drivers drivers;
    DjiMotor motor(&drivers, MOTOR1, tap::can::CanBus::CAN_BUS1, false, "cool motor");

    EXPECT_CALL(drivers.djiMotorTxHandler, addMotorToManager);
    EXPECT_CALL(drivers.canRxHandler, attachReceiveHandler);

    motor.initialize();
}

TEST(DjiMotor, parseCanRxData_invalid_motor_id)
{
    tap::Drivers drivers;
    DjiMotor motor(&drivers, MOTOR1, tap::can::CanBus::CAN_BUS1, false, "cool motor");

    modm::can::Message msg(MOTOR2, 8, {}, false);

    motor.processMessage(msg);

    EXPECT_FALSE(motor.isMotorOnline());
}

TEST(DjiMotor, parseCanRxData_valid_motor_id_motor_online)
{
    tap::arch::clock::ClockStub clock;
    tap::Drivers drivers;
    DjiMotor motor(&drivers, MOTOR1, tap::can::CanBus::CAN_BUS1, false, "cool motor");

    modm::can::Message msg(MOTOR1, 8, {}, false);

    motor.processMessage(msg);

    EXPECT_TRUE(motor.isMotorOnline());

    clock.time += 100'000;

    EXPECT_FALSE(motor.isMotorOnline());
}

struct MotorData
{
    uint16_t encoder;
    int16_t shaftRPM;
    int16_t torque;
    int8_t temperature;

    void encode(uint8_t *data)
    {
        data[0] = (encoder >> 8) & 0xff;
        data[1] = encoder & 0xff;
        data[2] = (shaftRPM >> 8) & 0xff;
        data[3] = shaftRPM & 0xff;
        data[4] = (torque >> 8) & 0xff;
        data[5] = torque & 0xff;
        data[6] = temperature;
    }
};

TEST(DjiMotor, parseCanRxData_motor_info_interpreted_correctly)
{
    tap::arch::clock::ClockStub clock;
    tap::Drivers drivers;
    DjiMotor motor(&drivers, MOTOR1, tap::can::CanBus::CAN_BUS1, false, "cool motor");

    modm::can::Message msg(MOTOR1, 8, {}, false);

    MotorData motorData;

    motorData.encoder = 1000;
    motorData.shaftRPM = -100;
    motorData.torque = 100;
    motorData.temperature = 43;

    motorData.encode(msg.data);

    motor.processMessage(msg);

    EXPECT_EQ(motorData.encoder, motor.getInternalEncoder()->getEncoderWrapped());
    EXPECT_EQ(motorData.encoder, motor.getInternalEncoder()->getEncoderUnwrapped());
    EXPECT_EQ(motorData.shaftRPM, motor.getShaftRPM());
    EXPECT_EQ(motorData.torque, motor.getTorque());
    EXPECT_EQ(motorData.temperature, motor.getTemperature());
}

TEST(DjiMotor, parseCanRxData_motor_info_interpreted_correctly_motor_inverted)
{
    tap::arch::clock::ClockStub clock;
    tap::Drivers drivers;
    DjiMotor motor(&drivers, MOTOR1, tap::can::CanBus::CAN_BUS1, true, "cool motor");

    modm::can::Message msg(MOTOR1, 8, {}, false);

    MotorData motorData;

    motorData.encoder = 1000;
    motorData.shaftRPM = -100;
    motorData.torque = 100;
    motorData.temperature = 43;

    motorData.encode(msg.data);

    motor.processMessage(msg);

    EXPECT_EQ(
        DjiMotor::ENC_RESOLUTION - motorData.encoder - 1,
        motor.getInternalEncoder()->getEncoderWrapped());
    EXPECT_EQ(
        DjiMotor::ENC_RESOLUTION - motorData.encoder - 1,
        motor.getInternalEncoder()->getEncoderUnwrapped());
    EXPECT_EQ(-motorData.shaftRPM, motor.getShaftRPM());
    EXPECT_EQ(-motorData.torque, motor.getTorque());
    EXPECT_EQ(motorData.temperature, motor.getTemperature());
}

TEST(DjiMotor, setDesiredOutput_limits_output)
{
    tap::Drivers drivers;
    DjiMotor motor(&drivers, MOTOR1, tap::can::CanBus::CAN_BUS1, false, "cool motor");

    motor.setDesiredOutput(static_cast<int32_t>(SHRT_MAX) + 1);
    EXPECT_EQ(SHRT_MAX, motor.getOutputDesired());

    motor.setDesiredOutput(static_cast<int32_t>(SHRT_MIN) - 1);
    EXPECT_EQ(SHRT_MIN, motor.getOutputDesired());
}

TEST(DjiMotor, isMotorInverted)
{
    tap::Drivers drivers;
    DjiMotor motorNotInverted(&drivers, MOTOR1, tap::can::CanBus::CAN_BUS1, false, "not inverted");
    DjiMotor motorInverted(&drivers, MOTOR2, tap::can::CanBus::CAN_BUS1, true, "inverted");

    EXPECT_FALSE(motorNotInverted.isMotorInverted());
    EXPECT_TRUE(motorInverted.isMotorInverted());
}

TEST(DjiMotor, serializeCanSendData_serializes_desired_output_in_correct_position)
{
    tap::Drivers drivers;
    DjiMotor motor(&drivers, MOTOR3, tap::can::CanBus::CAN_BUS1, false, "cool motor");

    motor.setDesiredOutput(1'000);

    modm::can::Message msg(MOTOR1, 8, 0xffff'ffff'ffff'ffff, false);

    motor.serializeCanSendData(&msg);

    int16_t serializedDesiredOutput =
        (static_cast<int16_t>(msg.data[4]) << 8) | static_cast<int16_t>(msg.data[5]);

    EXPECT_EQ(serializedDesiredOutput, motor.getOutputDesired());
}

TEST(DjiMotor, resetEncoderValue_zeroes_encoder_fields)
{
    tap::Drivers drivers;
    DjiMotor motor(&drivers, MOTOR1, tap::can::CanBus::CAN_BUS1, false, "cool motor");

    modm::can::Message msg(MOTOR1, 8, {}, false);

    MotorData motorData;

    motorData.encoder = 1000;
    motorData.shaftRPM = -100;
    motorData.torque = 100;
    motorData.temperature = 43;

    motorData.encode(msg.data);
    motor.processMessage(msg);

    motor.getEncoder()->resetEncoderValue();
    EXPECT_EQ(0, motor.getInternalEncoder()->getEncoderUnwrapped());
    EXPECT_EQ(0, motor.getInternalEncoder()->getEncoderWrapped());

    motorData.encoder = 8000;
    motorData.encode(msg.data);
    motor.processMessage(msg);

    motor.getEncoder()->resetEncoderValue();
    EXPECT_EQ(0, motor.getInternalEncoder()->getEncoderUnwrapped());
    EXPECT_EQ(0, motor.getInternalEncoder()->getEncoderWrapped());
}

TEST(DjiMotor, moving_relative_to_home_after_zeroed_ok)
{
    tap::Drivers drivers;
    DjiMotor motor(&drivers, MOTOR1, tap::can::CanBus::CAN_BUS1, false, "cool motor");

    static constexpr uint16_t ENC_RESOLUTION = 8192;

    modm::can::Message msg(MOTOR1, 8, {}, false);

    MotorData motorData;

    motorData.encoder = 1000;
    motorData.shaftRPM = -100;
    motorData.torque = 100;
    motorData.temperature = 43;

    motorData.encode(msg.data);
    motor.processMessage(msg);
    EXPECT_EQ(1000, motor.getInternalEncoder()->getEncoderUnwrapped());
    EXPECT_EQ(1000, motor.getInternalEncoder()->getEncoderWrapped());

    motor.getEncoder()->resetEncoderValue();
    EXPECT_EQ(0, motor.getInternalEncoder()->getEncoderUnwrapped());
    EXPECT_EQ(0, motor.getInternalEncoder()->getEncoderWrapped());

    motorData.encoder = 5000;
    motorData.encode(msg.data);
    motor.processMessage(msg);
    EXPECT_EQ(4000, motor.getInternalEncoder()->getEncoderUnwrapped());
    EXPECT_EQ(4000, motor.getInternalEncoder()->getEncoderWrapped());

    motorData.encoder = 2500;
    motorData.encode(msg.data);
    motor.processMessage(msg);
    EXPECT_EQ(1500, motor.getInternalEncoder()->getEncoderUnwrapped());
    EXPECT_EQ(1500, motor.getInternalEncoder()->getEncoderWrapped());

    motorData.encoder = 500;
    motorData.encode(msg.data);
    motor.processMessage(msg);
    EXPECT_EQ(-500, motor.getInternalEncoder()->getEncoderUnwrapped());
    EXPECT_EQ(ENC_RESOLUTION - 500, motor.getInternalEncoder()->getEncoderWrapped());
}

TEST(DjiMotor, inverted_moving_relative_to_home_after_zeroed_ok)
{
    tap::Drivers drivers;
    DjiMotor motor(&drivers, MOTOR1, tap::can::CanBus::CAN_BUS1, true, "inverted motor");

    static constexpr uint16_t ENC_RESOLUTION = 8192;

    modm::can::Message msg(MOTOR1, 8, {}, false);

    MotorData motorData;

    motorData.encoder = 1000;
    motorData.shaftRPM = -100;
    motorData.torque = 100;
    motorData.temperature = 43;

    motorData.encode(msg.data);
    motor.processMessage(msg);
    EXPECT_EQ(ENC_RESOLUTION - 1001, motor.getInternalEncoder()->getEncoderUnwrapped());
    EXPECT_EQ(ENC_RESOLUTION - 1001, motor.getInternalEncoder()->getEncoderWrapped());

    motor.getEncoder()->resetEncoderValue();
    EXPECT_EQ(0, motor.getInternalEncoder()->getEncoderUnwrapped());
    EXPECT_EQ(0, motor.getInternalEncoder()->getEncoderWrapped());

    motorData.encoder = 5000;
    motorData.encode(msg.data);
    motor.processMessage(msg);
    EXPECT_EQ(-4000, motor.getInternalEncoder()->getEncoderUnwrapped());
    EXPECT_EQ(4192, motor.getInternalEncoder()->getEncoderWrapped());

    motorData.encoder = 2500;
    motorData.encode(msg.data);
    motor.processMessage(msg);
    EXPECT_EQ(-1500, motor.getInternalEncoder()->getEncoderUnwrapped());
    EXPECT_EQ(6692, motor.getInternalEncoder()->getEncoderWrapped());

    motorData.encoder = 500;
    motorData.encode(msg.data);
    motor.processMessage(msg);
    EXPECT_EQ(500, motor.getInternalEncoder()->getEncoderUnwrapped());
    EXPECT_EQ(500, motor.getInternalEncoder()->getEncoderWrapped());
}
