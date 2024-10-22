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

#include "move_unjam_integral_comprised_command.hpp"

#include <cassert>

using namespace tap::control;

namespace tap::control::setpoint
{
MoveUnjamIntegralComprisedCommand::MoveUnjamIntegralComprisedCommand(
    tap::Drivers &drivers,
    IntegrableSetpointSubsystem &subsystem,
    MoveIntegralCommand &moveIntegralCommand,
    UnjamCommandInterface &unjamCommand)
    : tap::control::ComprisedCommand(&drivers),
      subsystem(subsystem),
      moveIntegralCommand(moveIntegralCommand),
      unjamCommand(unjamCommand),
      unjamSequenceCommencing(false)
{
    comprisedCommandScheduler.registerSubsystem(&subsystem);
    addSubsystemRequirement(&subsystem);

    assert(
        moveIntegralCommand.getRequirementsBitwise() == this->getRequirementsBitwise() &&
        unjamCommand.getRequirementsBitwise() == this->getRequirementsBitwise());
}

bool MoveUnjamIntegralComprisedCommand::isReady()
{
    return subsystem.isJammed() ? unjamCommand.isReady() : moveIntegralCommand.isReady();
}

void MoveUnjamIntegralComprisedCommand::initialize()
{
    comprisedCommandScheduler.addCommand(&moveIntegralCommand);
    unjamSequenceCommencing = false;
}

void MoveUnjamIntegralComprisedCommand::execute()
{
    if (subsystem.isJammed() && !unjamSequenceCommencing)
    {
        // when the setpointSubsystem is jammed, add the unjamCommand
        // the to scheduler. The rotate forward command will be automatically
        // unscheduled.
        unjamSequenceCommencing = true;
        comprisedCommandScheduler.addCommand(&unjamCommand);
    }
    comprisedCommandScheduler.run();
}

void MoveUnjamIntegralComprisedCommand::end(bool interrupted)
{
    comprisedCommandScheduler.removeCommand(&unjamCommand, interrupted);
    comprisedCommandScheduler.removeCommand(&moveIntegralCommand, interrupted);
}

bool MoveUnjamIntegralComprisedCommand::isFinished() const
{
    return (!unjamSequenceCommencing &&
            !comprisedCommandScheduler.isCommandScheduled(&moveIntegralCommand)) ||
           (unjamSequenceCommencing &&
            !comprisedCommandScheduler.isCommandScheduled(&unjamCommand));
}

}  // namespace tap::control::setpoint
