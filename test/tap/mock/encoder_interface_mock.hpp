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

#ifndef TAPROOT_DJI_MOTOR_ENCODER_MOCK_HPP_
#define TAPROOT_DJI_MOTOR_ENCODER_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/motor/encoder_interface.hpp"

namespace tap::mock
{
class EncoderInterfaceMock : public tap::motor::EncoderInterface
{
public:
    EncoderInterfaceMock();
    virtual ~EncoderInterfaceMock();

    MOCK_METHOD(void, initialize, (), (override));

    MOCK_METHOD(bool, isOnline, (), (const override));

    MOCK_METHOD(tap::algorithms::WrappedFloat, getPosition, (), (const override));

    MOCK_METHOD(float, getVelocity, (), (const override));

    MOCK_METHOD(int64_t, getEncoderUnwrapped, (), (const));

    MOCK_METHOD(uint16_t, getEncoderWrapped, (), (const));

    MOCK_METHOD(int16_t, getShaftRPM, (), (const));

    MOCK_METHOD(void, resetEncoderValue, (), (override));

    MOCK_METHOD(void, alignWith, (EncoderInterface*), (override));
};

}  // namespace tap::mock

#endif  // TAPROOT_DJI_MOTOR_ENCODER_MOCK_HPP_
