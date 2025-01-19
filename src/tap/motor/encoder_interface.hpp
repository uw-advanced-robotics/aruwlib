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

#ifndef TAPROOT_ENCODER_INTERFACE_HPP_
#define TAPROOT_ENCODER_INTERFACE_HPP_

#include "tap/algorithms/wrapped_float.hpp"

namespace tap::motor
{
class EncoderInterface
{
public:
    virtual void initialize() = 0;
    virtual bool isOnline() const = 0;
    virtual void resetEncoderValue() = 0;
    virtual tap::algorithms::WrappedFloat getPosition() const = 0;
    // rad/s
    virtual float getVelocity() const = 0;
};

}  // namespace tap::motor

#endif  // TAPROOT_ENCODER_INTERFACE_HPP_
