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

#ifndef TAPROOT_MOTOR_INTERFACE_HPP_
#define TAPROOT_MOTOR_INTERFACE_HPP_

#include <cstdint>

#include "encoder_interface.hpp"

namespace tap::motor
{
class MotorInterface
{
public:
    virtual void initialize() = 0;
    virtual EncoderInterface* getEncoder() const = 0;
    virtual void setDesiredOutput(int32_t desiredOutput) = 0;
    virtual bool isMotorOnline() const = 0;
    virtual int16_t getOutputDesired() const = 0;
    virtual int8_t getTemperature() const = 0;
    virtual int16_t getTorque() const = 0;
    
    [[deprecated]] virtual void resetEncoderValue() { this->getEncoder()->resetEncoderValue(); };
    [[deprecated]] virtual float getPositionUnwrapped() const { return this->getEncoder()->getPosition().getUnwrappedValue(); };
    [[deprecated]] virtual float getPositionWrapped() const { return this->getEncoder()->getPosition().getWrappedValue(); };
    [[deprecated]] virtual int16_t getShaftRPM() const { return this->getEncoder()->getVelocity() / static_cast<float>(M_TWOPI) * 60.f; };
};

}  // namespace tap::motor

#endif  // TAPROOT_MOTOR_INTERFACE_HPP_
