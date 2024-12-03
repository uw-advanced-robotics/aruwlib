/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "dji_motor_encoder.hpp"

#include "tap/algorithms/math_user_utils.hpp"

namespace tap
{
namespace motor
{
DjiMotorEncoder::DjiMotorEncoder(
    bool isInverted,
    uint16_t encoderWrapped,
    int64_t encoderRevolutions)
    : motorInverted(isInverted),
      encoderWrapped(encoderWrapped),
      encoderRevolutions(encoderRevolutions),
      encoderHomePosition(0)
{
    encoderDisconnectTimeout.stop();
}

void DjiMotorEncoder::processMessage(const modm::can::Message& message)
{
    encoderDisconnectTimeout.restart(MOTOR_DISCONNECT_TIME);
    uint16_t encoderActual =
        static_cast<uint16_t>(message.data[0] << 8 | message.data[1]);  // encoder value

    // invert motor if necessary
    encoderActual = motorInverted ? ENC_RESOLUTION - 1 - encoderActual : encoderActual;

    int32_t encoderRelativeToHome = (int32_t)encoderActual - (int32_t)encoderHomePosition;

    updateEncoderValue(
        encoderRelativeToHome < 0 ? (int32_t)ENC_RESOLUTION + encoderRelativeToHome
                                  : encoderRelativeToHome);
}

bool DjiMotorEncoder::isOnline() const
{
    /*
     * motor online if the disconnect timout has not expired (if it received message but
     * somehow got disconnected) and the timeout hasn't been stopped (initially, the timeout
     * is stopped)
     */
    return !encoderDisconnectTimeout.isExpired() && !encoderDisconnectTimeout.isStopped();
}

void DjiMotorEncoder::resetEncoderValue()
{
    encoderRevolutions = 0;
    encoderHomePosition = (encoderWrapped + encoderHomePosition) % ENC_RESOLUTION;
    encoderWrapped = 0;
}

float DjiMotorEncoder::getPositionUnwrapped() const
{
    return getEncoderUnwrapped() * M_TWOPI / ENC_RESOLUTION;
}

float DjiMotorEncoder::getPositionWrapped() const
{
    return getEncoderWrapped() * M_TWOPI / ENC_RESOLUTION;
}

int64_t DjiMotorEncoder::getEncoderUnwrapped() const
{
    return static_cast<int64_t>(encoderWrapped) +
           static_cast<int64_t>(ENC_RESOLUTION) * encoderRevolutions;
}

uint16_t DjiMotorEncoder::getEncoderWrapped() const { return encoderWrapped; }

void DjiMotorEncoder::updateEncoderValue(uint16_t newEncWrapped)
{
    int16_t enc_dif = newEncWrapped - encoderWrapped;
    if (enc_dif < -ENC_RESOLUTION / 2)
    {
        encoderRevolutions++;
    }
    else if (enc_dif > ENC_RESOLUTION / 2)
    {
        encoderRevolutions--;
    }
    encoderWrapped = newEncWrapped;
}
}  // namespace motor

}  // namespace tap
