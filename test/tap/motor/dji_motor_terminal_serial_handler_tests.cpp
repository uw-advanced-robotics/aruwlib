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
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/dji_motor_terminal_serial_handler.hpp"
#include "tap/stub/terminal_device_stub.hpp"

using namespace tap::motor;
using namespace testing;
using namespace tap;
using namespace testing;

TEST(DjiMotorTerminalSerialHandler, init__adds_itself_to_terminal_serial)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    EXPECT_CALL(drivers.terminalSerial, addHeader(_, &serialHandler));

    serialHandler.init();
}

TEST(DjiMotorTerminalSerialHandler, terminalSerialCallback__no_valid_input_returns_false)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char input[] = "\t sadf  \r \n sdf sdfkl ";
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input, stream, false));

    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), HasSubstr("Usage"));
}

TEST(DjiMotorTerminalSerialHandler, terminalSerialCallback__H_prints_usage)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char input[] = "-H";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), HasSubstr("Usage"));
}

TEST(DjiMotorTerminalSerialHandler, terminalSerialCallback__H_and_streaming_enabled_returns_false)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char input[] = "-H";
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input, stream, true));

    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), HasSubstr("Usage"));
}

TEST(DjiMotorTerminalSerialHandler, terminalSerialCallback__multiple_arguments_returns_usage)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char input1[] = "all -H";
    char input2[] = "-H all";
    char input3[] = "all motor";
    char input4[] = "motor 1 all";
    char input5[] = "motor 1 can 1 -H";
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input1, stream, false));
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input2, stream, false));
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input3, stream, false));
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input4, stream, false));
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input5, stream, false));
}

TEST(
    DjiMotorTerminalSerialHandler,
    terminalSerialCallback__motor_1_prints_non_nullptr_motors_on_bus_1_and_2)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    DjiMotor m1(&drivers, MotorId::MOTOR7, tap::can::CanBus::CAN_BUS1, false, "cool motor");
    DjiMotor m2(&drivers, MotorId::MOTOR7, tap::can::CanBus::CAN_BUS2, true, "coolest motorino");
    ON_CALL(drivers.djiMotorTxHandler, getCan1Motor).WillByDefault([&](tap::motor::MotorId mid) {
        return (mid == MotorId::MOTOR7) ? &m1 : nullptr;
    });
    ON_CALL(drivers.djiMotorTxHandler, getCan2Motor).WillByDefault([&](tap::motor::MotorId mid) {
        return (mid == MotorId::MOTOR7) ? &m2 : nullptr;
    });

    char input1[] = "motor 7";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input1, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("CAN 1"));
    EXPECT_THAT(output, HasSubstr("CAN 2"));
    EXPECT_THAT(output, HasSubstr("7. cool motor"));
    EXPECT_THAT(output, HasSubstr("7. coolest motorino"));

    char input2[] = "motor 2";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input2, stream, false));

    output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, Not(HasSubstr("2.")));
}

TEST(
    DjiMotorTerminalSerialHandler,
    terminalSerialCallback__can_1_prints_all_non_nullptr_motors_on_can1)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    DjiMotor *motors[DjiMotorTxHandler::DJI_MOTORS_PER_CAN] = {};
    motors[DJI_MOTOR_TO_NORMALIZED_ID(MotorId::MOTOR1)] =
        new DjiMotor(&drivers, MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "m1");
    motors[DJI_MOTOR_TO_NORMALIZED_ID(MotorId::MOTOR3)] =
        new DjiMotor(&drivers, MotorId::MOTOR3, tap::can::CanBus::CAN_BUS1, false, "m2");
    motors[DJI_MOTOR_TO_NORMALIZED_ID(MotorId::MOTOR6)] =
        new DjiMotor(&drivers, MotorId::MOTOR6, tap::can::CanBus::CAN_BUS1, false, "m3");

    ON_CALL(drivers.djiMotorTxHandler, getCan1Motor).WillByDefault([&](tap::motor::MotorId mid) {
        return motors[DJI_MOTOR_TO_NORMALIZED_ID(mid)];
    });

    char input[] = "can 1";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("1. m1"));
    EXPECT_THAT(output, HasSubstr("3. m2"));
    EXPECT_THAT(output, HasSubstr("6. m3"));

    delete motors[DJI_MOTOR_TO_NORMALIZED_ID(MotorId::MOTOR1)];
    delete motors[DJI_MOTOR_TO_NORMALIZED_ID(MotorId::MOTOR3)];
    delete motors[DJI_MOTOR_TO_NORMALIZED_ID(MotorId::MOTOR6)];
}

TEST(
    DjiMotorTerminalSerialHandler,
    terminalSerialCallback__can_3_or_motor_0_or_motor_9_or_motor_neg1_returns_false)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char input1[] = "can 3";
    char input2[] = "motor 0";
    char input3[] = "motor 9";
    char input4[] = "motor -1";
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input1, stream, false));
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input2, stream, false));
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input3, stream, false));
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input4, stream, false));
}

TEST(
    DjiMotorTerminalSerialHandler,
    terminalSerialCallback__all_prints_only_can_buses_when_no_motor_info)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char input[] = "all";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("CAN 1"));
    EXPECT_THAT(output, HasSubstr("CAN 2"));
}

TEST(DjiMotorTerminalSerialHandler, terminalSerialCallback__all_prints_all_motor_info)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    DjiMotor m1(&drivers, MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "cool motor");
    DjiMotor m2(&drivers, MotorId::MOTOR3, tap::can::CanBus::CAN_BUS2, true, "coolest motorino");
    ON_CALL(drivers.djiMotorTxHandler, getCan1Motor).WillByDefault([&](tap::motor::MotorId mid) {
        return (mid == MotorId::MOTOR1) ? &m1 : nullptr;
    });
    ON_CALL(drivers.djiMotorTxHandler, getCan2Motor).WillByDefault([&](tap::motor::MotorId mid) {
        return (mid == MotorId::MOTOR3) ? &m2 : nullptr;
    });

    char input[] = "all";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("1. cool motor"));
    EXPECT_THAT(output, HasSubstr("3. coolest motorino"));
}

TEST(DjiMotorTerminalSerialHandler, terminalSerialStreamCallback__prints_motor_info_last_specified)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    DjiMotor m1(&drivers, MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "cool motor");
    ON_CALL(drivers.djiMotorTxHandler, getCan1Motor).WillByDefault([&](tap::motor::MotorId mid) {
        return (mid == MotorId::MOTOR1) ? &m1 : nullptr;
    });

    char input[] = "motor 1 can 1";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, true));

    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), HasSubstr("1. cool motor"));

    serialHandler.terminalSerialStreamCallback(stream);

    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), HasSubstr("1. cool motor"));
}

TEST(DjiMotorTerminalSerialHandler, terminalSerialStreamCallback__prints_usage_when_blank_input)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char input[] = "";
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input, stream, false));

    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), HasSubstr("Usage"));
}

TEST(
    DjiMotorTerminalSerialHandler,
    terminalSerialCallback__can_2_prints_all_non_nullptr_motors_on_can2)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    DjiMotor *motors[DjiMotorTxHandler::DJI_MOTORS_PER_CAN] = {};
    motors[DJI_MOTOR_TO_NORMALIZED_ID(MotorId::MOTOR1)] =
        new DjiMotor(&drivers, MotorId::MOTOR1, tap::can::CanBus::CAN_BUS2, false, "m1");
    motors[DJI_MOTOR_TO_NORMALIZED_ID(MotorId::MOTOR3)] =
        new DjiMotor(&drivers, MotorId::MOTOR3, tap::can::CanBus::CAN_BUS2, false, "m2");
    motors[DJI_MOTOR_TO_NORMALIZED_ID(MotorId::MOTOR6)] =
        new DjiMotor(&drivers, MotorId::MOTOR6, tap::can::CanBus::CAN_BUS2, false, "m3");

    ON_CALL(drivers.djiMotorTxHandler, getCan2Motor).WillByDefault([&](tap::motor::MotorId mid) {
        return motors[DJI_MOTOR_TO_NORMALIZED_ID(mid)];
    });

    char input[] = "can 2";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("1. m1"));
    EXPECT_THAT(output, HasSubstr("3. m2"));
    EXPECT_THAT(output, HasSubstr("6. m3"));

    delete motors[DJI_MOTOR_TO_NORMALIZED_ID(MotorId::MOTOR1)];
    delete motors[DJI_MOTOR_TO_NORMALIZED_ID(MotorId::MOTOR3)];
    delete motors[DJI_MOTOR_TO_NORMALIZED_ID(MotorId::MOTOR6)];
}

TEST(
    DjiMotorTerminalSerialHandler,
    terminalSerialCallback__motor1_can_2_prints_info_about_specific_motor)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    DjiMotor *motors[DjiMotorTxHandler::DJI_MOTORS_PER_CAN] = {};
    motors[DJI_MOTOR_TO_NORMALIZED_ID(MotorId::MOTOR1)] =
        new DjiMotor(&drivers, MotorId::MOTOR1, tap::can::CanBus::CAN_BUS2, false, "m1");

    ON_CALL(drivers.djiMotorTxHandler, getCan2Motor).WillByDefault([&](tap::motor::MotorId mid) {
        return motors[DJI_MOTOR_TO_NORMALIZED_ID(mid)];
    });

    char input[] = "motor 1 can 2";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("1. m1"));

    delete motors[DJI_MOTOR_TO_NORMALIZED_ID(MotorId::MOTOR1)];
}

TEST(DjiMotorTerminalSerialHandler, terminalSerialCallback__invalid_can_id)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char input[] = "can 3";
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("Invalid can bus ID"));
}

TEST(DjiMotorTerminalSerialHandler, terminalSerialCallback__no_can_id)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char input[] = "can";
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("must specify can bus"));
}

TEST(DjiMotorTerminalSerialHandler, terminalSerialCallback__no_motor_id)
{
    Drivers drivers;
    DjiMotorTerminalSerialHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char input[] = "motor";
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("must specify motor id"));
}
