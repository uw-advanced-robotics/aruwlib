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

#ifndef TAPROOT_UNJAM_COMMAND_INTERFACE_HPP_
#define TAPROOT_UNJAM_COMMAND_INTERFACE_HPP_

#include <cstdint>

#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"

#include "../interfaces/integrable_setpoint_subsystem.hpp"

namespace tap::control::setpoint
{
/**
 * Command that attempts to unjam an agitator subsystem.
 */
class UnjamCommandInterface : public tap::control::Command
{
};

}  // namespace tap::control::setpoint

#endif  // TAPROOT_UNJAM_COMMAND_INTERFACE_HPP_
