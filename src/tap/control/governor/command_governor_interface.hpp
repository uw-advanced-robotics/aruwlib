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

#ifndef TAPROOT_COMMAND_GOVERNOR_INTERFACE_HPP_
#define TAPROOT_COMMAND_GOVERNOR_INTERFACE_HPP_

namespace tap::control::governor
{
/**
 * An interface that is used to gate the execution of a Command. Override this interface to gate
 * various commands based on some conditional logic. For example, create a sub-class of this
 * interface and have isReady return true when the ref system indicates you have enough heat to
 * launch a projectile. Then, use a GovernorLimitedCommand to only run a command that launches
 * a projectile when the CommandGovernorInterface sub-object you created is true.
 */
class CommandGovernorInterface
{
public:
    /// Returns true if the Command being governed by the governor may execute.
    virtual bool isReady() = 0;
    /// Returns true if the Command being governed by the governor should stop executing.
    virtual bool isFinished() = 0;
};
}  // namespace tap::control::governor

#endif  // TAPROOT_COMMAND_GOVERNOR_INTERFACE_HPP_
