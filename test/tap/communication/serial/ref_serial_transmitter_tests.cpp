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

#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/drivers.hpp"

using namespace tap::communication::serial;
using namespace tap;
using namespace testing;

class RefSerialTransmitterTest : public Test
{
protected:
    RefSerialTransmitterTest() : refSerialTransmitter(&drivers) {}

    void SetUp() override
    {
        ON_CALL(drivers.refSerial, getRobotData()).WillByDefault(ReturnRef(robotData));
    }

    template <typename T>
    void fillGraphicData(T &msg)
    {
        for (size_t i = 0; i < MODM_ARRAY_SIZE(msg.graphicData); i++)
        {
            uint8_t name[] = {0, 0, 0};

            RefSerialTransmitter::configGraphicGenerics(
                &msg.graphicData[i],
                name,
                RefSerialData::Tx::GraphicOperation::GRAPHIC_ADD,
                0,
                RefSerialData::Tx::GraphicColor::BLACK);

            RefSerialTransmitter::configLine(10, 0, 0, 42, 42, &msg.graphicData[i]);
        }
    }

    template <typename T>
    void sendGraphicNormalTest()
    {
        // Given
        robotData.robotId = RefSerial::RobotId::BLUE_SOLDIER_1;
        T msg{};
        fillGraphicData(msg);

        ON_CALL(drivers.refSerial, acquireTransmissionSemaphore).WillByDefault(Return(true));

        // Expected
        EXPECT_CALL(drivers.uart, write(_, _, sizeof(T))).WillOnce(Return(true));

        // When
        refSerialTransmitter.sendGraphic(&msg);
    }

    Drivers drivers;
    RefSerialTransmitter refSerialTransmitter;
    RefSerialData::Rx::RobotData robotData{};
};

TEST_F(RefSerialTransmitterTest, configGraphicGenerics__sets_name_operation_layer_color)
{
    RefSerialData::Tx::GraphicData data;

    uint8_t name[3] = {0, 1, 0};

    RefSerialTransmitter::configGraphicGenerics(
        &data,
        name,
        RefSerialData::Tx::GraphicOperation::GRAPHIC_MODIFY,
        0,
        RefSerialData::Tx::GraphicColor::PINK);

    EXPECT_TRUE(0 == std::memcmp(name, data.name, sizeof(name)));
    EXPECT_EQ(RefSerialData::Tx::GraphicOperation::GRAPHIC_MODIFY, data.operation);
    EXPECT_EQ(static_cast<uint8_t>(RefSerialData::Tx::GraphicColor::PINK), data.color);
}

TEST_F(RefSerialTransmitterTest, configLine__sets_params)
{
    RefSerialData::Tx::GraphicData data;

    RefSerialTransmitter::configLine(100, 50, 20, 75, 40, &data);

    EXPECT_EQ(100, data.lineWidth);
    EXPECT_EQ(50, data.startX);
    EXPECT_EQ(20, data.startY);
    EXPECT_EQ(75, data.endX);
    EXPECT_EQ(40, data.endY);
}

TEST_F(RefSerialTransmitterTest, configRectangle__sets_params)
{
    RefSerialData::Tx::GraphicData data;

    RefSerialTransmitter::configRectangle(100, 50, 20, 75, 40, &data);

    EXPECT_EQ(100, data.lineWidth);
    EXPECT_EQ(50, data.startX);
    EXPECT_EQ(20, data.startY);
    EXPECT_EQ(75, data.endX);
    EXPECT_EQ(40, data.endY);
}

TEST_F(RefSerialTransmitterTest, configCircle__sets_params)
{
    RefSerialData::Tx::GraphicData data;

    RefSerialTransmitter::configCircle(100, 50, 20, 75, &data);

    EXPECT_EQ(100, data.lineWidth);
    EXPECT_EQ(50, data.startX);
    EXPECT_EQ(20, data.startY);
    EXPECT_EQ(75, data.radius);
}

TEST_F(RefSerialTransmitterTest, configEllipse__sets_params)
{
    RefSerialData::Tx::GraphicData data;

    RefSerialTransmitter::configEllipse(100, 50, 20, 75, 40, &data);

    EXPECT_EQ(100, data.lineWidth);
    EXPECT_EQ(50, data.startX);
    EXPECT_EQ(20, data.startY);
    EXPECT_EQ(75, data.endX);
    EXPECT_EQ(40, data.endY);
}

TEST_F(RefSerialTransmitterTest, configArc__sets_params)
{
    RefSerialData::Tx::GraphicData data;

    RefSerialTransmitter::configArc(0, 10, 23, 50, 20, 75, 40, &data);

    EXPECT_EQ(0, data.startAngle);
    EXPECT_EQ(10, data.endAngle);
    EXPECT_EQ(23, data.lineWidth);
    EXPECT_EQ(50, data.startX);
    EXPECT_EQ(20, data.startY);
    EXPECT_EQ(75, data.endX);
    EXPECT_EQ(40, data.endY);
}

TEST_F(RefSerialTransmitterTest, configCharMsg__msg_len_lt_max_len)
{
    RefSerialData::Tx::GraphicCharacterMessage data;

    const char msg[] = "hello";

    RefSerialTransmitter::configCharacterMsg(10, 2, 45, 54, msg, &data);

    EXPECT_EQ(10, data.graphicData.startAngle);
    EXPECT_EQ(2, data.graphicData.lineWidth);
    EXPECT_EQ(5, data.graphicData.endAngle);
    EXPECT_EQ(45, data.graphicData.startX);
    EXPECT_EQ(54, data.graphicData.startY);
    EXPECT_EQ(std::string(msg), std::string(data.msg));
}

TEST_F(RefSerialTransmitterTest, deleteGraphicLayer__doesnt_send_if_robot_id_invalid)
{
    RefSerialData::Tx::DeleteGraphicOperation op =
        RefSerialData::Tx::DeleteGraphicOperation::DELETE_GRAPHIC_LAYER;

    EXPECT_CALL(drivers.uart, write(testing::_, testing::_, testing::_)).Times(0);

    refSerialTransmitter.deleteGraphicLayer(op, 1);
}

TEST_F(RefSerialTransmitterTest, deleteGraphicLayer__blocks_on_transmission_semaphore)
{
    robotData.robotId = RefSerial::RobotId::RED_SOLDIER_3;

    EXPECT_CALL(drivers.refSerial, acquireTransmissionSemaphore)
        .Times(2)
        .WillOnce(Return(false))
        .WillRepeatedly(Return(true));

    EXPECT_CALL(drivers.uart, write(testing::_, testing::_, testing::_));

    refSerialTransmitter.deleteGraphicLayer(
        RefSerialData::Tx::DeleteGraphicOperation::DELETE_GRAPHIC_LAYER,
        2);
}

TEST_F(RefSerialTransmitterTest, deleteGraphicLayer__sends_correct_msg)
{
    // Given
    robotData.robotId = RefSerial::RobotId::RED_SOLDIER_3;

    ON_CALL(drivers.refSerial, acquireTransmissionSemaphore).WillByDefault(Return(true));

    // Expected
    EXPECT_CALL(drivers.uart, write(testing::_, testing::_, testing::_))
        .WillOnce([&](tap::communication::serial::Uart::UartPort,
                      const uint8_t *data,
                      std::size_t length) {
            const RefSerialData::Tx::DeleteGraphicLayerMessage *msg =
                reinterpret_cast<const RefSerialData::Tx::DeleteGraphicLayerMessage *>(data);

            EXPECT_EQ(
                sizeof(msg->interactiveHeader) + sizeof(msg->deleteOperation) + sizeof(msg->layer),
                msg->frameHeader.dataLength);
            uint16_t cmdId = *reinterpret_cast<const uint16_t *>(data + sizeof(msg->frameHeader));
            EXPECT_EQ(0x0301, cmdId);
            EXPECT_EQ(0xa5, msg->frameHeader.headByte);
            EXPECT_EQ(
                tap::algorithms::calculateCRC8(data, sizeof(msg->frameHeader) - 1),
                msg->frameHeader.CRC8);

            EXPECT_EQ(0x0100, msg->interactiveHeader.dataCmdId);
            EXPECT_EQ(
                0x0100 + static_cast<uint16_t>(RefSerial::RobotId::RED_SOLDIER_3),
                msg->interactiveHeader.receiverId);
            EXPECT_EQ(
                static_cast<uint16_t>(RefSerial::RobotId::RED_SOLDIER_3),
                msg->interactiveHeader.senderId);

            EXPECT_EQ(0x0301, msg->cmdId);
            EXPECT_EQ(
                static_cast<uint16_t>(
                    RefSerialData::Tx::DeleteGraphicOperation::DELETE_GRAPHIC_LAYER),
                msg->deleteOperation);
            EXPECT_EQ(2, msg->layer);

            return length;
        });

    // When
    refSerialTransmitter.deleteGraphicLayer(
        RefSerialData::Tx::DeleteGraphicOperation::DELETE_GRAPHIC_LAYER,
        2);
}

TEST_F(RefSerialTransmitterTest, deleteGraphicLayer__releases_lock)
{
    tap::arch::clock::ClockStub clock;
    robotData.robotId = RefSerial::RobotId::RED_SOLDIER_3;

    ON_CALL(drivers.refSerial, acquireTransmissionSemaphore).WillByDefault(Return(true));
    EXPECT_CALL(drivers.refSerial, releaseTransmissionSemaphore);

    refSerialTransmitter.deleteGraphicLayer(
        RefSerialData::Tx::DeleteGraphicOperation::DELETE_GRAPHIC_LAYER,
        2);
}

TEST_F(RefSerialTransmitterTest, sendGraphic__1_doesnt_send_if_robot_id_invalid)
{
    RefSerialData::Tx::Graphic1Message msg{};

    EXPECT_CALL(drivers.uart, write(testing::_, testing::_, testing::_)).Times(0);

    refSerialTransmitter.sendGraphic(&msg);
}

TEST_F(RefSerialTransmitterTest, sendGraphic__1_doesnt_send_but_configures_if_sendMsg_false)
{
    // Given
    robotData.robotId = RefSerial::RobotId::BLUE_SOLDIER_1;

    RefSerialData::Tx::Graphic1Message msg{};

    ON_CALL(drivers.refSerial, acquireTransmissionSemaphore).WillByDefault(Return(true));

    // Expected
    EXPECT_CALL(drivers.uart, write(testing::_, testing::_, testing::_)).Times(0);

    // When
    refSerialTransmitter.sendGraphic(&msg, true, false);

    // Then
    // validate the msg header was still constructed
    EXPECT_EQ(0x0301, msg.cmdId);
    EXPECT_EQ(sizeof(msg.graphicData) + sizeof(msg.interactiveHeader), msg.frameHeader.dataLength);
    EXPECT_EQ(0xa5, msg.frameHeader.headByte);
    EXPECT_EQ(
        static_cast<uint16_t>(RefSerial::RobotId::BLUE_SOLDIER_1),
        msg.interactiveHeader.senderId);
    EXPECT_EQ(
        0x100 + static_cast<uint16_t>(RefSerial::RobotId::BLUE_SOLDIER_1),
        msg.interactiveHeader.receiverId);
}

TEST_F(RefSerialTransmitterTest, sendGraphic__2_normal)
{
    sendGraphicNormalTest<RefSerialData::Tx::Graphic2Message>();
}

TEST_F(RefSerialTransmitterTest, sendGraphic_5_normal)
{
    sendGraphicNormalTest<RefSerialData::Tx::Graphic5Message>();
}

TEST_F(RefSerialTransmitterTest, sendGraphic_7_normal)
{
    sendGraphicNormalTest<RefSerialData::Tx::Graphic7Message>();
}

TEST_F(RefSerialTransmitterTest, sendGraphic__characterMessage)
{
    // Given
    robotData.robotId = RefSerial::RobotId::BLUE_ENGINEER;

    RefSerialData::Tx::GraphicCharacterMessage msg{};

    msg.msg[0] = 'h';
    msg.msg[1] = 'e';
    msg.msg[2] = 'l';
    msg.msg[3] = 'l';
    msg.msg[4] = 'o';
    msg.msg[5] = '\0';

    ON_CALL(drivers.refSerial, acquireTransmissionSemaphore).WillByDefault(Return(true));

    // Expect
    EXPECT_CALL(
        drivers.uart,
        write(testing::_, testing::_, sizeof(RefSerialData::Tx::GraphicCharacterMessage)))
        .WillOnce([&](tap::communication::serial::Uart::UartPort,
                      const uint8_t *data,
                      std::size_t length) {
            const auto header = reinterpret_cast<const RefSerial::FrameHeader *>(data);
            EXPECT_EQ(
                sizeof(msg.interactiveHeader) + sizeof(msg.graphicData) + sizeof(msg.msg),
                header->dataLength);

            uint16_t cmdId = *reinterpret_cast<const uint16_t *>(data + sizeof(msg.frameHeader));
            EXPECT_EQ(0x0301, cmdId);

            const auto interactiveHeader =
                reinterpret_cast<const RefSerialData::Tx::InteractiveHeader *>(
                    data + sizeof(msg.frameHeader) + sizeof(msg.cmdId));
            EXPECT_EQ(0x0110, interactiveHeader->dataCmdId);
            EXPECT_EQ(
                RefSerial::RobotId::BLUE_ENGINEER,
                static_cast<RefSerial::RobotId>(interactiveHeader->senderId));
            EXPECT_EQ(
                0x100 + static_cast<uint16_t>(RefSerial::RobotId::BLUE_ENGINEER),
                interactiveHeader->receiverId);

            // Don't care about graphic data, only msg

            const uint8_t *msgData = data + sizeof(msg.frameHeader) + sizeof(msg.cmdId) +
                                     sizeof(msg.interactiveHeader) + sizeof(msg.graphicData);
            for (size_t i = 0; i < 6; i++)
            {
                EXPECT_EQ(msg.msg[i], msgData[i]);
            }

            // Validate crc16
            EXPECT_EQ(
                tap::algorithms::calculateCRC16(
                    data,
                    sizeof(RefSerialData::Tx::GraphicCharacterMessage) - sizeof(uint16_t)),
                *reinterpret_cast<const uint16_t *>(
                    data + sizeof(RefSerialData::Tx::GraphicCharacterMessage) - sizeof(uint16_t)));

            return length;
        });

    // When
    refSerialTransmitter.sendGraphic(&msg);
}

TEST_F(RefSerialTransmitterTest, sendRobotToRobotMessage__msgLen_too_short_fails_to_send)
{
    robotData.robotId = RefSerial::RobotId::INVALID;

    RefSerialData::Tx::RobotToRobotMessage msg{};

    EXPECT_CALL(drivers.errorController, addToErrorList)
        .Times(2)
        .WillRepeatedly([](const tap::errors::SystemError &error) {
            EXPECT_TRUE(errorDescriptionContainsSubstr(error, "message length cannot be 1 byte"));
        });

    EXPECT_CALL(drivers.uart, write(_, _, _)).Times(0);

    refSerialTransmitter.sendRobotToRobotMsg(&msg, 0x0200, RefSerial::RobotId::RED_HERO, 1);
    refSerialTransmitter.sendRobotToRobotMsg(&msg, 0x0200, RefSerial::RobotId::RED_HERO, 1);
}

TEST_F(RefSerialTransmitterTest, sendRobotToRobotMessage__invalid_id_fails_to_send)
{
    // Given
    robotData.robotId = RefSerial::RobotId::INVALID;

    RefSerialData::Tx::RobotToRobotMessage msg{};

    ON_CALL(drivers.refSerial, acquireTransmissionSemaphore).WillByDefault(Return(true));

    // Expected
    EXPECT_CALL(drivers.errorController, addToErrorList)
        .Times(2)
        .WillRepeatedly([](const tap::errors::SystemError &error) {
            EXPECT_TRUE(
                errorDescriptionContainsSubstr(error, "invalid msgId not between [0x200, 0x2ff)"));
        });

    EXPECT_CALL(drivers.uart, write(_, _, _)).Times(0);

    // When
    refSerialTransmitter.sendRobotToRobotMsg(&msg, 0x01ff, RefSerial::RobotId::RED_HERO, 2);
    refSerialTransmitter.sendRobotToRobotMsg(&msg, 0x1000, RefSerial::RobotId::RED_HERO, 2);
}

TEST_F(RefSerialTransmitterTest, sendRobotToRobotMessage__msgLen_too_long)
{
    // Given
    robotData.robotId = RefSerial::RobotId::BLUE_SOLDIER_1;

    RefSerialData::Tx::RobotToRobotMessage msg{};

    ON_CALL(drivers.refSerial, acquireTransmissionSemaphore).WillByDefault(Return(true));

    // Expected
    EXPECT_CALL(drivers.errorController, addToErrorList)
        .WillOnce([](const tap::errors::SystemError &error) {
            EXPECT_TRUE(errorDescriptionContainsSubstr(error, "message length > 113-char maximum"));
        });

    EXPECT_CALL(drivers.uart, write(_, _, _)).Times(0);

    // When
    refSerialTransmitter.sendRobotToRobotMsg(&msg, 0x0200, RefSerial::RobotId::BLUE_SOLDIER_2, 150);
}

TEST_F(RefSerialTransmitterTest, sendRobotToRobotMessage__invalid_robot_id)
{
    // Given
    robotData.robotId = RefSerial::RobotId::INVALID;

    RefSerialData::Tx::RobotToRobotMessage msg{};

    // Expected
    EXPECT_CALL(drivers.uart, write(_, _, _)).Times(0);

    // When
    refSerialTransmitter.sendRobotToRobotMsg(&msg, 0x0200, RefSerial::RobotId::BLUE_SOLDIER_2, 10);
}

// @todo implement equivalent delay test in ref_serial_tests.cpp
// TEST_F(RefSerialTransmitterTest,
// sendRobotToRobotMessage__locks_and_releases_transmission_semaphore)
// {
//     tap::arch::clock::ClockStub clock;

//     robotData.robotId = RefSerial::RobotId::RED_DRONE;

//     RefSerialData::Tx::RobotToRobotMessage msg;

//     msg.dataAndCRC16[0] = 'h';
//     msg.dataAndCRC16[1] = 'i';

//     static constexpr int msgLen = 2;

//     static constexpr int entireMsgLen = sizeof(msg.frameHeader) + sizeof(msg.cmdId) +
//                                         sizeof(msg.interactiveHeader) + msgLen +
//                                         sizeof(uint16_t);

//     EXPECT_CALL(drivers.uart, write(testing::_, testing::_, entireMsgLen));

//     bool lockAcquired = false;
//     EXPECT_CALL(drivers.refSerial, acquireTransmissionSemaphore).WillOnce([&]() {
//         lockAcquired = true;
//         return true;
//     });
//     EXPECT_CALL(drivers.refSerial, releaseTransmissionSemaphore).WillOnce([&]() {
//         lockAcquired = false;
//     });

//     refSerialTransmitter.sendRobotToRobotMsg(&msg, 0x0200, RefSerial::RobotId::RED_HERO, 2);

//     EXPECT_TRUE(lockAcquired);

//     clock.time += 100'000;

//     refSerialTransmitter.sendRobotToRobotMsg(&msg, 0x0200, RefSerial::RobotId::RED_HERO, 2);

//     EXPECT_FALSE(lockAcquired);
// }

TEST_F(
    RefSerialTransmitterTest,
    sendRobotToRobotMessage__validate_sending_msg_to_same_color_robot_works)
{
    // Given
    robotData.robotId = RefSerial::RobotId::RED_DRONE;

    RefSerialData::Tx::RobotToRobotMessage msg;

    msg.dataAndCRC16[0] = 'h';
    msg.dataAndCRC16[1] = 'i';

    static constexpr int msgLen = 2;

    static constexpr int entireMsgLen = sizeof(msg.frameHeader) + sizeof(msg.cmdId) +
                                        sizeof(msg.interactiveHeader) + msgLen + sizeof(uint16_t);

    ON_CALL(drivers.refSerial, acquireTransmissionSemaphore).WillByDefault(Return(true));

    // Expect
    EXPECT_CALL(drivers.uart, write(testing::_, testing::_, entireMsgLen))
        .WillOnce([&](tap::communication::serial::Uart::UartPort,
                      const uint8_t *data,
                      std::size_t length) {
            // Decode and validate header
            const RefSerial::FrameHeader *header =
                reinterpret_cast<const RefSerial::FrameHeader *>(data);
            EXPECT_EQ(sizeof(msg.interactiveHeader) + msgLen, header->dataLength);
            EXPECT_EQ(0xa5, header->headByte);
            EXPECT_EQ(
                tap::algorithms::calculateCRC8(data, sizeof(RefSerial::FrameHeader) - 1),
                header->CRC8);

            // Decode and validate interactive header
            const RefSerialData::Tx::InteractiveHeader *interactiveHeader =
                reinterpret_cast<const RefSerialData::Tx::InteractiveHeader *>(
                    data + sizeof(msg.frameHeader) + sizeof(msg.cmdId));
            EXPECT_EQ(0x0200, interactiveHeader->dataCmdId);
            EXPECT_EQ(
                RefSerial::RobotId::RED_HERO,
                static_cast<RefSerial::RobotId>(interactiveHeader->receiverId));
            EXPECT_EQ(
                RefSerial::RobotId::RED_DRONE,
                static_cast<RefSerial::RobotId>(interactiveHeader->senderId));

            // Decode and validate message
            static constexpr int START_DATA_OFFSET =
                sizeof(msg.frameHeader) + sizeof(msg.cmdId) + sizeof(msg.interactiveHeader);
            EXPECT_EQ('h', data[START_DATA_OFFSET]);
            EXPECT_EQ('i', data[START_DATA_OFFSET + 1]);

            // Validate crc16
            EXPECT_EQ(
                tap::algorithms::calculateCRC16(data, entireMsgLen - sizeof(uint16_t)),
                *reinterpret_cast<const uint16_t *>(data + entireMsgLen - sizeof(uint16_t)));
            return length;
        });

    // When
    refSerialTransmitter.sendRobotToRobotMsg(&msg, 0x0200, RefSerial::RobotId::RED_HERO, 2);
}
