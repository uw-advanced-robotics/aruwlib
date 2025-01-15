/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/architecture/endianness_wrappers.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/dji_motor_mock.hpp"
#include "tap/motor/dji_motor_tx_handler.hpp"

using namespace testing;
using namespace tap;
using namespace tap::motor;
using namespace tap::mock;
using namespace tap::arch;

class DjiMotorTxHandlerTest : public Test
{
protected:
    DjiMotorTxHandlerTest() : drivers(), djiMotorTxHandler(&drivers), motors()
    {
        for (size_t i = 0; i < DjiMotorTxHandler::DJI_MOTORS_PER_CAN; i++)
        {
            if (i % 2 == 0) {
                motors.emplace_back(new NiceMock<DjiMotorMock>(
                &drivers,
                NORMALIZED_ID_TO_DJI_MOTOR(i),
                can::CanBus::CAN_BUS1,
                false,
                ""));
            } else {
                motors.emplace_back(new NiceMock<DjiMotorMock>(
                &drivers,
                NORMALIZED_ID_TO_DJI_MOTOR(i),
                can::CanBus::CAN_BUS1,
                false,
                "",
                1000,
                0,
                true));
            }
            
        }

        for (size_t i = 0; i < DjiMotorTxHandler::DJI_MOTORS_PER_CAN; i++)
        {
            if (i % 2 == 0) {
                motors.emplace_back(new NiceMock<DjiMotorMock>(
                &drivers,
                NORMALIZED_ID_TO_DJI_MOTOR(i),
                can::CanBus::CAN_BUS2,
                false,
                ""));
            } else {
                motors.emplace_back(new NiceMock<DjiMotorMock>(
                &drivers,
                NORMALIZED_ID_TO_DJI_MOTOR(i),
                can::CanBus::CAN_BUS2,
                false,
                "",
                1000,
                0,
                true));
            }
            
        }
    }

    ~DjiMotorTxHandlerTest()
    {
        for (auto element : motors)
        {
            delete element;
        }
    }

    void SetUp() override
    {
        ON_CALL(drivers.can, isReadyToSend).WillByDefault(Return(true));
        ON_CALL(drivers.can, sendMessage).WillByDefault(Return(true));

        for (auto *motor : motors)
        {
            ON_CALL(*motor, getMotorIdentifier).WillByDefault([motor]() {
                return motor->DjiMotor::getMotorIdentifier();
            });

            ON_CALL(*motor, getCanBus).WillByDefault([motor]() {
                return motor->DjiMotor::getCanBus();
            });
        }
    }

    void TearDown() override {}

    void addAllMotors()
    {
        for (auto motor : motors)
        {
            djiMotorTxHandler.addMotorToManager(motor);
        }
    }

    Drivers drivers;
    DjiMotorTxHandler djiMotorTxHandler;

    std::vector<NiceMock<DjiMotorMock> *> motors;
};

TEST_F(DjiMotorTxHandlerTest, DJI_MOTOR_TO_NORMALIZED_ID_motor1_idx_0)
{
    EXPECT_EQ(0, DJI_MOTOR_TO_NORMALIZED_ID(MOTOR1));
}

TEST_F(DjiMotorTxHandlerTest, DJI_MOTOR_TO_NORMALIZED_ID_motor8_idx_7)
{
    EXPECT_EQ(7, DJI_MOTOR_TO_NORMALIZED_ID(MOTOR8));
}

TEST_F(DjiMotorTxHandlerTest, NORMALIZED_ID_TO_DJI_MOTOR_idx0_motor1)
{
    EXPECT_EQ(MOTOR1, NORMALIZED_ID_TO_DJI_MOTOR(0));
}

TEST_F(DjiMotorTxHandlerTest, NORMALIZED_ID_TO_DJI_MOTOR_idx7_motor8)
{
    EXPECT_EQ(MOTOR8, NORMALIZED_ID_TO_DJI_MOTOR(7));
}

TEST_F(DjiMotorTxHandlerTest, addMotorToManager_multiple_unique_motor_ids)
{
    // adding these motors should not result in crashing
    djiMotorTxHandler.addMotorToManager(motors[0]);
    djiMotorTxHandler.addMotorToManager(motors[1]);
    djiMotorTxHandler.addMotorToManager(motors[2]);
}

TEST_F(DjiMotorTxHandlerTest, removeFromMotorManager_motor_not_added_errors)
{
    EXPECT_CALL(drivers.errorController, addToErrorList);

    DjiMotor m1(&drivers, motor::MOTOR1, can::CanBus::CAN_BUS1, false, "hi");
    DjiMotor m2(&drivers, motor::MOTOR2, can::CanBus::CAN_BUS1, false, "hi");

    djiMotorTxHandler.addMotorToManager(&m1);

    djiMotorTxHandler.removeFromMotorManager(m2);
}

TEST_F(DjiMotorTxHandlerTest, removeFromMotorManager_invalid_motor_id_errors)
{
    EXPECT_CALL(drivers.errorController, addToErrorList).Times(2);

    ON_CALL(*motors[0], getMotorIdentifier).WillByDefault(Return(motor::MOTOR1 - 1));
    ON_CALL(*motors[1], getMotorIdentifier).WillByDefault(Return(motor::MOTOR8 + 1));

    djiMotorTxHandler.removeFromMotorManager(*motors[0]);
    djiMotorTxHandler.removeFromMotorManager(*motors[1]);
}

TEST_F(DjiMotorTxHandlerTest, encodeAndSendCanData_no_motors_added_no_can_messages_sent)
{
    EXPECT_CALL(drivers.can, sendMessage).Times(0);

    djiMotorTxHandler.encodeAndSendCanData();
}

TEST_F(DjiMotorTxHandlerTest, encodeAndSendCanData_single_motor_added_single_message_sent)
{
    djiMotorTxHandler.addMotorToManager(motors[0]);

    EXPECT_CALL(drivers.can, sendMessage).Times(1);

    djiMotorTxHandler.encodeAndSendCanData();
}

TEST_F(DjiMotorTxHandlerTest, encodeAndSendCanData_all_motors_added_6_messages_sent)
{
    EXPECT_CALL(drivers.can, sendMessage).Times(6);

    addAllMotors();

    djiMotorTxHandler.encodeAndSendCanData();
}

TEST_F(DjiMotorTxHandlerTest, encodeAndSendCanData_error_if_sendMessage_fails)
{
    ON_CALL(drivers.can, sendMessage).WillByDefault(Return(false));

    EXPECT_CALL(drivers.errorController, addToErrorList);

    addAllMotors();

    djiMotorTxHandler.encodeAndSendCanData();
}

TEST_F(DjiMotorTxHandlerTest, encodeAndSendCanData_does_not_send_if_can_bus_busy)
{
    ON_CALL(drivers.can, isReadyToSend).WillByDefault(Return(false));

    addAllMotors();

    djiMotorTxHandler.encodeAndSendCanData();
}

TEST_F(DjiMotorTxHandlerTest, encodeAndSendCanData_valid_encoding)
{
    modm::can::Message can1MessageLow(
        DjiMotorTxHandler::CAN_DJI_LOW_IDENTIFIER,
        DjiMotorTxHandler::CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);
    convertToLittleEndian<int16_t>(1, can1MessageLow.data);
    modm::can::Message can1MessageHigh(
        DjiMotorTxHandler::CAN_DJI_HIGH_IDENTIFIER,
        DjiMotorTxHandler::CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);
    convertToLittleEndian<int16_t>(2, can1MessageHigh.data);
    modm::can::Message can1Message6020Current(
        DjiMotorTxHandler::CAN_DJI_6020_CURRENT_IDENTIFIER,
        DjiMotorTxHandler::CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);
    convertToLittleEndian<int16_t>(3, can1Message6020Current.data);
    modm::can::Message can2MessageLow(
        DjiMotorTxHandler::CAN_DJI_LOW_IDENTIFIER,
        DjiMotorTxHandler::CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);
    convertToLittleEndian<int16_t>(4, can2MessageLow.data);
    modm::can::Message can2MessageHigh(
        DjiMotorTxHandler::CAN_DJI_HIGH_IDENTIFIER,
        DjiMotorTxHandler::CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);
    convertToLittleEndian<int16_t>(5, can2MessageHigh.data);
    modm::can::Message can2Message6020Current(
        DjiMotorTxHandler::CAN_DJI_6020_CURRENT_IDENTIFIER,
        DjiMotorTxHandler::CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);
    convertToLittleEndian<int16_t>(6, can2Message6020Current.data);

    ON_CALL(*motors[0], serializeCanSendData).WillByDefault([](modm::can::Message *txMessage) {
        convertToLittleEndian(1, txMessage->data);
    });
    ON_CALL(*motors[4], serializeCanSendData).WillByDefault([](modm::can::Message *txMessage) {
        convertToLittleEndian(2, txMessage->data);
    });
    ON_CALL(*motors[8], serializeCanSendData).WillByDefault([](modm::can::Message *txMessage) {
        convertToLittleEndian(4, txMessage->data);
    });
    ON_CALL(*motors[12], serializeCanSendData).WillByDefault([](modm::can::Message *txMessage) {
        convertToLittleEndian(5, txMessage->data);
    });
    ON_CALL(*motors[16], serializeCanSendData).WillByDefault([](modm::can::Message *txMessage) {
        convertToLittleEndian(3, txMessage->data);
    });
    ON_CALL(*motors[24], serializeCanSendData).WillByDefault([](modm::can::Message *txMessage) {
        convertToLittleEndian(6, txMessage->data);
    });

    EXPECT_CALL(drivers.can, sendMessage(can::CanBus::CAN_BUS1, can1MessageLow));
    EXPECT_CALL(drivers.can, sendMessage(can::CanBus::CAN_BUS1, can1MessageHigh));
    EXPECT_CALL(drivers.can, sendMessage(can::CanBus::CAN_BUS2, can2MessageLow));
    EXPECT_CALL(drivers.can, sendMessage(can::CanBus::CAN_BUS2, can2MessageHigh));
    EXPECT_CALL(drivers.can, sendMessage(can::CanBus::CAN_BUS1, can1Message6020Current));
    EXPECT_CALL(drivers.can, sendMessage(can::CanBus::CAN_BUS2, can2Message6020Current));

    // sending info about single motor on can1/2 low/high
    djiMotorTxHandler.addMotorToManager(motors[0]);
    djiMotorTxHandler.addMotorToManager(motors[4]);
    djiMotorTxHandler.addMotorToManager(motors[8]);
    djiMotorTxHandler.addMotorToManager(motors[12]);
    djiMotorTxHandler.addMotorToManager(motors[16]);
    djiMotorTxHandler.addMotorToManager(motors[24]);

    djiMotorTxHandler.encodeAndSendCanData();
}

#define TEST_getCanNMotor(n)                                                              \
    TEST_F(DjiMotorTxHandlerTest, getCan##n##Motor_returns_nullptr_when_invalid_motorid)  \
    {                                                                                     \
        EXPECT_EQ(                                                                        \
            nullptr,                                                                      \
            djiMotorTxHandler.getCan##n##Motor(static_cast<MotorId>(motor::MOTOR8 + 1))); \
        EXPECT_EQ(                                                                        \
            nullptr,                                                                      \
            djiMotorTxHandler.getCan##n##Motor(static_cast<MotorId>(motor::MOTOR1 - 1))); \
    }                                                                                     \
    TEST_F(                                                                               \
        DjiMotorTxHandlerTest,                                                            \
        getCan##n##Motor_returns_nullptr_when_motor_id_not_in_motor_manager)              \
    {                                                                                     \
        EXPECT_EQ(                                                                        \
            nullptr,                                                                      \
            djiMotorTxHandler.getCan##n##Motor(static_cast<MotorId>(motor::MOTOR1)));     \
    }

TEST_getCanNMotor(1);

TEST_getCanNMotor(2);

TEST_F(DjiMotorTxHandlerTest, getCan1Motor_returns_proper_motor_when_valid_motor_id)
{
    djiMotorTxHandler.addMotorToManager(motors[0]);
    EXPECT_EQ(
        motors[0],
        djiMotorTxHandler.getCan1Motor(
            static_cast<motor::MotorId>(motors[0]->DjiMotor::getMotorIdentifier())));
}

TEST_F(DjiMotorTxHandlerTest, getCan2Motor_returns_proper_motor_when_valid_motor_id)
{
    djiMotorTxHandler.addMotorToManager(motors[8]);
    EXPECT_EQ(
        motors[8],
        djiMotorTxHandler.getCan2Motor(
            static_cast<motor::MotorId>(motors[8]->DjiMotor::getMotorIdentifier())));
}
