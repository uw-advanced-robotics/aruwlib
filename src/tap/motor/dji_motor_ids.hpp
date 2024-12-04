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

#ifndef TAPROOT_DJI_MOTOR_IDS_HPP_
#define TAPROOT_DJI_MOTOR_IDS_HPP_

#include <cstdint>

namespace tap::motor
{
/**
 * CAN IDs for the feedback messages sent by DJI motor controllers. Motor `i` in the set
 * {1, 2,...,8} sends feedback data with in a CAN message with ID 0x200 + `i`.
 * for declaring a new motor, must be one of these motor
 * identifiers
 */
enum MotorId : uint32_t
{
    MOTOR1 = 0X201,
    MOTOR2 = 0x202,
    MOTOR3 = 0x203,
    MOTOR4 = 0x204,
    MOTOR5 = 0x205,
    MOTOR6 = 0x206,
    MOTOR7 = 0x207,
    MOTOR8 = 0x208,
};
}  // namespace tap::motor

#endif  // TAPROOT_DJI_MOTOR_IDS_HPP_
