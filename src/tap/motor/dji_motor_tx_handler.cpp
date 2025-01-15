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

#include "dji_motor_tx_handler.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "modm/architecture/interface/assert.h"
#include "modm/architecture/interface/can_message.hpp"

namespace tap::motor
{
void DjiMotorTxHandler::addMotorToManager(DjiMotor** canMotorStore, DjiMotor* const motor)
{
    assert(motor != nullptr);
    uint32_t idIndex = DJI_MOTOR_TO_NORMALIZED_ID(motor->getMotorIdentifier());
    bool motorOverloaded = canMotorStore[idIndex] != nullptr;
    bool motorOutOfBounds = idIndex >= DJI_MOTORS_PER_CAN;
    std::cout << "Adding motor " << idIndex << ", overloaded: " << motorOverloaded
              << ", out of bounds: " << motorOutOfBounds << std::endl;
    modm_assert(
        !motorOverloaded && !motorOutOfBounds,
        "DjiMotorTxHandler",
        "Adding too many motors to manager");
    canMotorStore[idIndex] = motor;
}

void DjiMotorTxHandler::addMotorToManager(DjiMotor* motor)
{
    // add new motor to either the can1 or can2 motor store
    // because we checked to see if the motor is overloaded, we will
    // never have to worry about overfilling the CanxMotorStore array
    if (motor->getCanBus() == tap::can::CanBus::CAN_BUS1)
    {
        std::cout << "Adding motor to CAN1" << std::endl;
        addMotorToManager(can1MotorStore, motor);
    }
    else
    {
        std::cout << "Adding motor to CAN2" << std::endl;
        addMotorToManager(can2MotorStore, motor);
    }
}

void DjiMotorTxHandler::encodeAndSendCanData()
{
    // set up new can messages to be sent via CAN bus 1 and 2
    modm::can::Message can1MessageLow(
        CAN_DJI_LOW_IDENTIFIER,
        CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);
    modm::can::Message can1MessageHigh(
        CAN_DJI_HIGH_IDENTIFIER,
        CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);
    modm::can::Message can1Message6020Current(
        CAN_DJI_6020_CURRENT_IDENTIFIER,
        CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);
    modm::can::Message can2MessageLow(
        CAN_DJI_LOW_IDENTIFIER,
        CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);
    modm::can::Message can2MessageHigh(
        CAN_DJI_HIGH_IDENTIFIER,
        CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);
    modm::can::Message can2Message6020Current(
        CAN_DJI_6020_CURRENT_IDENTIFIER,
        CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);

    bool can1ValidMotorMessageLow = false;
    bool can1ValidMotorMessageHigh = false;
    bool can1ValidMotorMessage6020Current = false;
    bool can2ValidMotorMessageLow = false;
    bool can2ValidMotorMessageHigh = false;
    bool can2ValidMotorMessage6020Current = false;

    serializeMotorStoreSendData(
        can1MotorStore,
        &can1MessageLow,
        &can1MessageHigh,
        &can1Message6020Current,
        &can1ValidMotorMessageLow,
        &can1ValidMotorMessageHigh,
        &can1ValidMotorMessage6020Current);

    serializeMotorStoreSendData(
        can2MotorStore,
        &can2MessageLow,
        &can2MessageHigh,
        &can2Message6020Current,
        &can2ValidMotorMessageLow,
        &can2ValidMotorMessageHigh,
        &can2ValidMotorMessage6020Current);

    bool messageSuccess = true;

    if (drivers->can.isReadyToSend(can::CanBus::CAN_BUS1))
    {
        if (can1ValidMotorMessageLow)
        {
            messageSuccess &= drivers->can.sendMessage(can::CanBus::CAN_BUS1, can1MessageLow);
        }
        if (can1ValidMotorMessageHigh)
        {
            messageSuccess &= drivers->can.sendMessage(can::CanBus::CAN_BUS1, can1MessageHigh);
        }
        if (can1ValidMotorMessage6020Current)
        {
            messageSuccess &=
                drivers->can.sendMessage(can::CanBus::CAN_BUS1, can1Message6020Current);
        }
    }
    if (drivers->can.isReadyToSend(can::CanBus::CAN_BUS2))
    {
        if (can2ValidMotorMessageLow)
        {
            messageSuccess &= drivers->can.sendMessage(can::CanBus::CAN_BUS2, can2MessageLow);
        }
        if (can2ValidMotorMessageHigh)
        {
            messageSuccess &= drivers->can.sendMessage(can::CanBus::CAN_BUS2, can2MessageHigh);
        }
        if (can2ValidMotorMessage6020Current)
        {
            messageSuccess &=
                drivers->can.sendMessage(can::CanBus::CAN_BUS2, can2Message6020Current);
        }
    }

    if (!messageSuccess)
    {
        RAISE_ERROR(drivers, "sendMessage failure");
    }
}

void DjiMotorTxHandler::serializeMotorStoreSendData(
    DjiMotor** canMotorStore,
    modm::can::Message* messageLow,
    modm::can::Message* messageHigh,
    modm::can::Message* message6020Current,
    bool* validMotorMessageLow,
    bool* validMotorMessageHigh,
    bool* validMotorMessage6020Current)
{
    for (int i = 0; i < DJI_MOTORS_PER_CAN; i++)
    {
        const DjiMotor* const motor = canMotorStore[i];
        if (motor != nullptr)
        {
            std::cout << "Motor " << DJI_MOTOR_TO_NORMALIZED_ID(motor->getMotorIdentifier());
            if (DJI_MOTOR_TO_NORMALIZED_ID(motor->getMotorIdentifier()) <=
                DJI_MOTOR_TO_NORMALIZED_ID(tap::motor::MOTOR4))
            {
                std::cout << " is in low" << std::endl;
                motor->serializeCanSendData(messageLow);
                *validMotorMessageLow = true;
            }
            else if (motor->isInCurrentControl())
            {
                std::cout << " is in 6020 current" << std::endl;
                motor->serializeCanSendData(message6020Current);
                *validMotorMessage6020Current = true;
            }
            else
            {
                std::cout << " is in high" << std::endl;
                motor->serializeCanSendData(messageHigh);
                *validMotorMessageHigh = true;
            }
        }
    }
}

void DjiMotorTxHandler::removeFromMotorManager(const DjiMotor& motor)
{
    if (motor.getCanBus() == tap::can::CanBus::CAN_BUS1)
    {
        removeFromMotorManager(motor, can1MotorStore);
    }
    else
    {
        removeFromMotorManager(motor, can2MotorStore);
    }
}

void DjiMotorTxHandler::removeFromMotorManager(const DjiMotor& motor, DjiMotor** motorStore)
{
    uint32_t id = DJI_MOTOR_TO_NORMALIZED_ID(motor.getMotorIdentifier());
    if (id > DJI_MOTOR_TO_NORMALIZED_ID(tap::motor::MOTOR8) || motorStore[id] == nullptr)
    {
        RAISE_ERROR(drivers, "invalid motor id");
        return;
    }
    motorStore[id] = nullptr;
}

DjiMotor const* DjiMotorTxHandler::getCan1Motor(MotorId motorId)
{
    uint32_t index = DJI_MOTOR_TO_NORMALIZED_ID(motorId);
    return index > DJI_MOTOR_TO_NORMALIZED_ID(tap::motor::MOTOR8) ? nullptr : can1MotorStore[index];
}

DjiMotor const* DjiMotorTxHandler::getCan2Motor(MotorId motorId)
{
    uint32_t index = DJI_MOTOR_TO_NORMALIZED_ID(motorId);
    return index > DJI_MOTOR_TO_NORMALIZED_ID(tap::motor::MOTOR8) ? nullptr : can2MotorStore[index];
}
}  // namespace tap::motor
