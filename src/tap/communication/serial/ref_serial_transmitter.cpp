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

#include "ref_serial_transmitter.hpp"

#include <cstring>

#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "ref_serial.hpp"

namespace tap::communication::serial
{

RefSerialTransmitter::RefSerialTransmitter(Drivers* drivers) : drivers(drivers) {}

void RefSerialTransmitter::configGraphicGenerics(
    Tx::GraphicData* graphicData,
    const uint8_t* name,
    Tx::GraphicOperation operation,
    uint8_t layer,
    Tx::GraphicColor color)
{
    memcpy(graphicData->name, name, 3);
    graphicData->operation = operation;
    graphicData->layer = layer;
    graphicData->color = static_cast<uint8_t>(color);
}

void RefSerialTransmitter::configLine(
    uint16_t width,
    uint16_t startX,
    uint16_t startY,
    uint16_t endX,
    uint16_t endY,
    Tx::GraphicData* sharedData)
{
    sharedData->type = static_cast<uint8_t>(Tx::GraphicType::STRAIGHT_LINE);
    sharedData->lineWidth = width;
    sharedData->startX = startX;
    sharedData->startY = startY;
    sharedData->endX = endX;
    sharedData->endY = endY;
}

void RefSerialTransmitter::configRectangle(
    uint16_t width,
    uint16_t startX,
    uint16_t startY,
    uint16_t endX,
    uint16_t endY,
    Tx::GraphicData* sharedData)
{
    sharedData->type = static_cast<uint8_t>(Tx::GraphicType::RECTANGLE);
    sharedData->lineWidth = width;
    sharedData->startX = startX;
    sharedData->startY = startY;
    sharedData->endX = endX;
    sharedData->endY = endY;
}

void RefSerialTransmitter::configCircle(
    uint16_t width,
    uint16_t centerX,
    uint16_t centerY,
    uint16_t radius,
    Tx::GraphicData* sharedData)
{
    sharedData->type = static_cast<uint8_t>(Tx::GraphicType::CIRCLE);
    sharedData->lineWidth = width;
    sharedData->startX = centerX;
    sharedData->startY = centerY;
    sharedData->radius = radius;
}

void RefSerialTransmitter::configEllipse(
    uint16_t width,
    uint16_t centerX,
    uint16_t centerY,
    uint16_t xLen,
    uint16_t yLen,
    Tx::GraphicData* sharedData)
{
    sharedData->type = static_cast<uint8_t>(Tx::GraphicType::ELLIPSE);
    sharedData->lineWidth = width;
    sharedData->startX = centerX;
    sharedData->startY = centerY;
    sharedData->endX = xLen;
    sharedData->endY = yLen;
}

void RefSerialTransmitter::configArc(
    uint16_t startAngle,
    uint16_t endAngle,
    uint16_t width,
    uint16_t centerX,
    uint16_t centerY,
    uint16_t xLen,
    uint16_t yLen,
    Tx::GraphicData* sharedData)
{
    sharedData->type = static_cast<uint8_t>(Tx::GraphicType::ARC);
    sharedData->startAngle = startAngle;
    sharedData->endAngle = endAngle;
    sharedData->lineWidth = width;
    sharedData->startX = centerX;
    sharedData->startY = centerY;
    sharedData->endX = xLen;
    sharedData->endY = yLen;
}

void RefSerialTransmitter::configFloatingNumber(
    uint16_t fontSize,
    uint16_t decimalPrecision,
    uint16_t width,
    uint16_t startX,
    uint16_t startY,
    float value,
    Tx::GraphicData* sharedData)
{
    sharedData->type = static_cast<uint8_t>(Tx::GraphicType::FLOATING_NUM);
    sharedData->startAngle = fontSize;
    sharedData->endAngle = decimalPrecision;
    sharedData->lineWidth = width;
    sharedData->startX = startX;
    sharedData->startY = startY;
    // Store floating point value in fixed point with 3 decimal points precision
    sharedData->value = 1000 * value;
}

void RefSerialTransmitter::configInteger(
    uint16_t fontSize,
    uint16_t width,
    uint16_t startX,
    uint16_t startY,
    int32_t value,
    Tx::GraphicData* sharedData)
{
    sharedData->type = static_cast<uint8_t>(Tx::GraphicType::INTEGER);
    sharedData->startAngle = fontSize;
    sharedData->lineWidth = width;
    sharedData->startX = startX;
    sharedData->startY = startY;
    sharedData->value = value;
}

void RefSerialTransmitter::configCharacterMsg(
    uint16_t fontSize,
    uint16_t width,
    uint16_t startX,
    uint16_t startY,
    const char* dataToPrint,
    Tx::GraphicCharacterMessage* sharedData)
{
    sharedData->graphicData.type = static_cast<uint8_t>(Tx::GraphicType::CHARACTER);
    sharedData->graphicData.startAngle = fontSize;
    sharedData->graphicData.endAngle = strlen(dataToPrint);
    sharedData->graphicData.lineWidth = width;
    sharedData->graphicData.startX = startX;
    sharedData->graphicData.startY = startY;
    strncpy(sharedData->msg, dataToPrint, MODM_ARRAY_SIZE(sharedData->msg) - 1);
}

/**
 * Given RobotId, returns the client_id that the referee system uses to display
 * the received messages to the given client_id robot.
 *
 * @param[in] robotId the id of the robot received from the referee system
 *      to get the client_id of.
 * @return the client_id of the robot requested.
 * 
 * @todo general send() method which acquires semaphore and does uart write?
 */
static uint16_t getRobotClientID(RefSerialTransmitter::RobotId robotId)
{
    return 0x100 + static_cast<uint16_t>(robotId);
}

modm::ResumableResult<void> RefSerialTransmitter::deleteGraphicLayer(
    Tx::DeleteGraphicOperation graphicOperation,
    uint8_t graphicLayer)
{
    RF_BEGIN(0);
    if (drivers->refSerial.getRobotData().robotId == RefSerialTransmitter::RobotId::INVALID)
    {
        RF_RETURN_1();
    }

    deleteGraphicLayerMessage.deleteOperation = graphicOperation;
    deleteGraphicLayerMessage.layer = graphicLayer;

    configFrameHeader(
        &deleteGraphicLayerMessage.frameHeader,
        sizeof(deleteGraphicLayerMessage.interactiveHeader) +
            sizeof(deleteGraphicLayerMessage.deleteOperation) +
            sizeof(deleteGraphicLayerMessage.layer));

    deleteGraphicLayerMessage.cmdId = RefSerial::REF_MESSAGE_TYPE_CUSTOM_DATA;

    configInteractiveHeader(
        &deleteGraphicLayerMessage.interactiveHeader,
        0x100,
        drivers->refSerial.getRobotData().robotId,
        getRobotClientID(drivers->refSerial.getRobotData().robotId));

    deleteGraphicLayerMessage.crc16 = algorithms::calculateCRC16(
        reinterpret_cast<uint8_t*>(&deleteGraphicLayerMessage),
        sizeof(Tx::DeleteGraphicLayerMessage) - sizeof(deleteGraphicLayerMessage.crc16));

    RF_WAIT_UNTIL(drivers->refSerial.acquireTransmissionSemaphore() && (delayTimer.isExpired() || delayTimer.isStopped()));

    drivers->uart.write(
        bound_ports::REF_SERIAL_UART_PORT,
        reinterpret_cast<uint8_t*>(&deleteGraphicLayerMessage),
        sizeof(Tx::DeleteGraphicLayerMessage));

    drivers->refSerial.releaseTransmissionSemaphore();

    delayTimer.restart(Tx::getWaitTimeAfterGraphicSendMs(&deleteGraphicLayerMessage));

    RF_END();
}

/**
 * A helper function used by the RefSerialTransmitter's sendGraphic functions to send some
 * GraphicType to the referee system. The user may specify whether or not to configure the message
 * header and whether or not the actually send the message. This helper function is needed because
 * the sendGraphic functions all send messages the same way with only minor differences.
 */
template<typename GRAPHIC>
modm::ResumableResult<void> RefSerialTransmitter::sendGraphic_(
    GRAPHIC* graphicMsg,
    uint16_t messageId,
    bool configMsgHeader,
    bool sendMsg,
    RefSerialTransmitter::RobotId robotId,
    tap::Drivers* drivers,
    uint8_t extraDataLength)
{
    RF_BEGIN(1);
    if (robotId == RefSerialTransmitter::RobotId::INVALID)
    {
        RF_RETURN_1();
    }
    if (configMsgHeader)
    {
        RefSerialTransmitter::configFrameHeader(
            &graphicMsg->frameHeader,
            sizeof(graphicMsg->graphicData) + sizeof(graphicMsg->interactiveHeader) +
                extraDataLength);
        graphicMsg->cmdId = RefSerial::REF_MESSAGE_TYPE_CUSTOM_DATA;
        RefSerialTransmitter::configInteractiveHeader(
            &graphicMsg->interactiveHeader,
            messageId,
            robotId,
            getRobotClientID(robotId));
        graphicMsg->crc16 = algorithms::calculateCRC16(
            reinterpret_cast<uint8_t*>(graphicMsg),
            sizeof(*graphicMsg) - sizeof(graphicMsg->crc16));
    }
    if (sendMsg)
    {
        RF_WAIT_UNTIL(drivers->refSerial.acquireTransmissionSemaphore() && (delayTimer.isExpired() || delayTimer.isStopped()));
        drivers->uart.write(
            bound_ports::REF_SERIAL_UART_PORT,
            reinterpret_cast<uint8_t*>(graphicMsg),
            sizeof(*graphicMsg));
        delayTimer.restart(Tx::getWaitTimeAfterGraphicSendMs(graphicMsg));
        drivers->refSerial.releaseTransmissionSemaphore();
    }
    RF_END();
}

modm::ResumableResult<void> RefSerialTransmitter::sendGraphic(
    Tx::Graphic1Message* graphicMsg,
    bool configMsgHeader,
    bool sendMsg)
{
    RF_BEGIN(2);
    RF_RETURN_CALL(sendGraphic_<Tx::Graphic1Message>(
        graphicMsg,
        0x101,
        configMsgHeader,
        sendMsg,
        drivers->refSerial.getRobotData().robotId,
        drivers,
        0));
    RF_END();
}

modm::ResumableResult<void> RefSerialTransmitter::sendGraphic(
    Tx::Graphic2Message* graphicMsg,
    bool configMsgHeader,
    bool sendMsg)
{
    RF_BEGIN(3);
    RF_RETURN_CALL(sendGraphic_<Tx::Graphic2Message>(
        graphicMsg,
        0x102,
        configMsgHeader,
        sendMsg,
        drivers->refSerial.getRobotData().robotId,
        drivers,
        0));
    RF_END();
}

modm::ResumableResult<void> RefSerialTransmitter::sendGraphic(
    Tx::Graphic5Message* graphicMsg,
    bool configMsgHeader,
    bool sendMsg)
{
    RF_BEGIN(4);
    RF_RETURN_CALL(sendGraphic_<Tx::Graphic5Message>(
        graphicMsg,
        0x103,
        configMsgHeader,
        sendMsg,
        drivers->refSerial.getRobotData().robotId,
        drivers,
        0));
    RF_END();
}

modm::ResumableResult<void> RefSerialTransmitter::sendGraphic(
    Tx::Graphic7Message* graphicMsg,
    bool configMsgHeader,
    bool sendMsg)
{
    RF_BEGIN(5);
    RF_RETURN_CALL(sendGraphic_<Tx::Graphic7Message>(
        graphicMsg,
        0x104,
        configMsgHeader,
        sendMsg,
        drivers->refSerial.getRobotData().robotId,
        drivers,
        0));
    RF_END();
}

modm::ResumableResult<void> RefSerialTransmitter::sendGraphic(
    Tx::GraphicCharacterMessage* graphicMsg,
    bool configMsgHeader,
    bool sendMsg)
{
    RF_BEGIN(6);
    RF_RETURN_CALL(sendGraphic_<Tx::GraphicCharacterMessage>(
        graphicMsg,
        0x110,
        configMsgHeader,
        sendMsg,
        drivers->refSerial.getRobotData().robotId,
        drivers,
        MODM_ARRAY_SIZE(graphicMsg->msg)));
    RF_END();
}

modm::ResumableResult<void> RefSerialTransmitter::sendRobotToRobotMsg(
    Tx::RobotToRobotMessage* robotToRobotMsg,
    uint16_t msgId,
    RobotId receiverId,
    uint16_t msgLen)
{
    RF_BEGIN(7);

    if (msgId < 0x0200 || msgId >= 0x02ff)
    {
        RAISE_ERROR(drivers, "invalid msgId not between [0x200, 0x2ff)");
        RF_RETURN_1();
    }

    if (msgLen > 113)
    {
        RAISE_ERROR(drivers, "message length > 113-char maximum");
        RF_RETURN_1();
    }

    if (drivers->refSerial.getRobotData().robotId == RobotId::INVALID)
    {
        RF_RETURN_1();
    }

    configFrameHeader(
        &robotToRobotMsg->frameHeader,
        sizeof(robotToRobotMsg->interactiveHeader) + msgLen);

    robotToRobotMsg->cmdId = RefSerial::REF_MESSAGE_TYPE_CUSTOM_DATA;

    configInteractiveHeader(
        &robotToRobotMsg->interactiveHeader,
        msgId,
        drivers->refSerial.getRobotData().robotId,
        static_cast<uint16_t>(receiverId));

    static constexpr int FULL_MSG_SIZE_LESS_MSGLEN = sizeof(robotToRobotMsg->frameHeader) +
                                                     sizeof(robotToRobotMsg->cmdId) +
                                                     sizeof(robotToRobotMsg->interactiveHeader);

    *reinterpret_cast<uint16_t*>(robotToRobotMsg->dataAndCRC16 + msgLen) =
        algorithms::calculateCRC16(
            reinterpret_cast<uint8_t*>(robotToRobotMsg),
            FULL_MSG_SIZE_LESS_MSGLEN + msgLen);

    RF_WAIT_UNTIL(drivers->refSerial.acquireTransmissionSemaphore() && (delayTimer.isExpired() || delayTimer.isStopped()));

    drivers->uart.write(
        bound_ports::REF_SERIAL_UART_PORT,
        reinterpret_cast<uint8_t*>(&robotToRobotMsg),
        FULL_MSG_SIZE_LESS_MSGLEN + msgLen + sizeof(uint16_t));

    delayTimer.restart(Tx::getWaitTimeAfterGraphicSendMs(robotToRobotMsg));

    drivers->refSerial.releaseTransmissionSemaphore();

    RF_END();
}

void RefSerialTransmitter::configFrameHeader(DJISerial::FrameHeader* header, uint16_t msgLen)
{
    header->headByte = 0xa5;
    header->dataLength = msgLen;
    header->seq = 0;
    header->CRC8 = algorithms::calculateCRC8(
        reinterpret_cast<const uint8_t*>(header),
        sizeof(DJISerial::FrameHeader) - sizeof(header->CRC8));
}

void RefSerialTransmitter::configInteractiveHeader(
    Tx::InteractiveHeader* header,
    uint16_t cmdId,
    RobotId senderId,
    uint16_t receiverId)
{
    header->dataCmdId = cmdId;
    header->senderId = static_cast<uint16_t>(senderId);
    header->receiverId = receiverId;
}
}  // namespace tap::communication::serial
