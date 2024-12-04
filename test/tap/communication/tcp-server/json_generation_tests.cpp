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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "tap/communication/can/can.hpp"
#include "tap/communication/tcp-server/json_messages.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/dji_motor_mock.hpp"

using tap::Drivers;
using tap::mock::DjiMotorMock;
using namespace tap::can;
using namespace tap::motor;
using namespace tap::communication::json;
using namespace testing;

TEST(JSONMessages, ReturnsCorrectString)
{
    Drivers drivers;
    DjiMotorMock mockMotor(&drivers, MotorId::MOTOR1, CanBus::CAN_BUS1, false, "MockMotor");
    EXPECT_CALL(mockMotor, getCanBus()).Times(1).WillOnce(Return(tap::can::CanBus::CAN_BUS2));
    EXPECT_CALL(mockMotor, getMotorIdentifier()).Times(1);
    EXPECT_CALL(mockMotor, getShaftRPM()).Times(1);
    EXPECT_CALL(mockMotor, getTorque()).Times(1);
    EXPECT_CALL(mockMotor->getInternalEncoder(), getEncoderUnwrapped()).Times(1);
    EXPECT_CALL(drivers.canRxHandler, removeReceiveHandler).Times(AnyNumber());

    std::string s = makeMotorMessage(mockMotor);

    EXPECT_EQ(
        "{\"messageType\":\"motor\",\"canBus\":2,\"motorID\":0,"
        "\"shaftRPM\":0,\"torque\":0,\"encoderValue\":0}",
        s);
}
