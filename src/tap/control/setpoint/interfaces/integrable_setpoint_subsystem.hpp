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

#ifndef TAPROOT_INTEGRABLE_SETPOINT_SUBSYSTEM_HPP_
#define TAPROOT_INTEGRABLE_SETPOINT_SUBSYSTEM_HPP_

#include "setpoint_subsystem.hpp"

namespace tap
{
// Forward declaration
class Drivers;
}  // namespace tap

namespace tap::control::setpoint
{
/**
 * An extension of the SetpointSubsystem. Identical to the SetpointSubsystem except that the
 * setpoint is assumed to be integrable. As such, an additional getCurrentValueIntegral abstract
 * function must be implemented by those who choose to extend this class.
 */
class IntegrableSetpointSubsystem : public virtual SetpointSubsystem
{
public:
    /**
     * @return The current integral value of the setpoint, a measurement with units `units *
     * seconds` (units of setpoint integrated with respect to time)
     */
    virtual float getCurrentValueIntegral() const = 0;
};

}  // namespace tap::control::setpoint

#endif  // TAPROOT_INTEGRABLE_SETPOINT_SUBSYSTEM_HPP_
