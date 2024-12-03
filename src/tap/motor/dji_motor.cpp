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

#include "dji_motor.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

#ifdef PLATFORM_HOSTED
#include <iostream>

#include "tap/communication/tcp-server/json_messages.hpp"
#include "tap/communication/tcp-server/tcp_server.hpp"

#include "modm/architecture/interface/can_message.hpp"
#endif

namespace tap
{
namespace motor
{
DjiMotor::~DjiMotor() { drivers->djiMotorTxHandler.removeFromMotorManager(*this); }

DjiMotor::DjiMotor(
    Drivers* drivers,
    MotorId desMotorIdentifier,
    tap::can::CanBus motorCanBus,
    bool isInverted,
    const char* name,
    uint16_t encoderWrapped,
    int64_t encoderRevolutions,
    EncoderInterface* externalEncoder)
    : CanRxListener(drivers, static_cast<uint32_t>(desMotorIdentifier), motorCanBus),
      motorName(name),
      drivers(drivers),
      motorIdentifier(desMotorIdentifier),
      motorCanBus(motorCanBus),
      desiredOutput(0),
      shaftRPM(0),
      temperature(0),
      torque(0),
      motorInverted(isInverted),
      internalEncoder(isInverted, encoderWrapped, encoderRevolutions),
      encoder(
          {externalEncoder != nullptr ? externalEncoder : &internalEncoder,
           externalEncoder != nullptr ? &internalEncoder : nullptr})
{
    motorDisconnectTimeout.stop();
}

void DjiMotor::initialize()
{
    drivers->djiMotorTxHandler.addMotorToManager(this);
    attachSelfToRxHandler();
    this->encoder.initialize();
}

void DjiMotor::processMessage(const modm::can::Message& message)
{
    if (message.getIdentifier() != DjiMotor::getMotorIdentifier())
    {
        return;
    }
    shaftRPM = static_cast<int16_t>(message.data[2] << 8 | message.data[3]);  // rpm
    shaftRPM = motorInverted ? -shaftRPM : shaftRPM;
    torque = static_cast<int16_t>(message.data[4] << 8 | message.data[5]);  // torque
    torque = motorInverted ? -torque : torque;
    temperature = static_cast<int8_t>(message.data[6]);  // temperature

    // restart disconnect timer, since you just received a message from the motor
    motorDisconnectTimeout.restart(MOTOR_DISCONNECT_TIME);

    this->internalEncoder.processMessage(message);
}

void DjiMotor::setDesiredOutput(int32_t desiredOutput)
{
    int16_t desOutputNotInverted =
        static_cast<int16_t>(tap::algorithms::limitVal<int32_t>(desiredOutput, SHRT_MIN, SHRT_MAX));
    this->desiredOutput = motorInverted ? -desOutputNotInverted : desOutputNotInverted;
}

bool DjiMotor::isMotorOnline() const
{
    /*
     * motor online if the disconnect timout has not expired (if it received message but
     * somehow got disconnected) and the timeout hasn't been stopped (initially, the timeout
     * is stopped)
     */
    return !motorDisconnectTimeout.isExpired() && !motorDisconnectTimeout.isStopped();
}

void DjiMotor::serializeCanSendData(modm::can::Message* txMessage) const
{
    int id = DJI_MOTOR_TO_NORMALIZED_ID(this->getMotorIdentifier());  // number between 0 and 7
    // this method assumes you have choosen the correct message
    // to send the data in. Is blind to message type and is a private method
    // that I use accordingly.
    id %= 4;
    txMessage->data[2 * id] = this->getOutputDesired() >> 8;
    txMessage->data[2 * id + 1] = this->getOutputDesired() & 0xFF;
}

// getter functions
int16_t DjiMotor::getOutputDesired() const { return desiredOutput; }

uint32_t DjiMotor::getMotorIdentifier() const { return motorIdentifier; }

int8_t DjiMotor::getTemperature() const { return temperature; }

int16_t DjiMotor::getTorque() const { return torque; }

int16_t DjiMotor::getShaftRPM() const { return shaftRPM; }

bool DjiMotor::isMotorInverted() const { return motorInverted; }

tap::can::CanBus DjiMotor::getCanBus() const { return motorCanBus; }

const char* DjiMotor::getName() const { return motorName; }

void DjiMotor::resetEncoderValue() { this->encoder.resetEncoderValue(); }

float DjiMotor::getPositionUnwrapped() const { return this->encoder.getPositionUnwrapped(); }

float DjiMotor::getPositionWrapped() const { return this->encoder.getPositionWrapped(); }
}  // namespace motor

}  // namespace tap
