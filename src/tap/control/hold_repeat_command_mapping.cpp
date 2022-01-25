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

#include "hold_repeat_command_mapping.hpp"

#include "tap/drivers.hpp"

namespace tap
{
namespace control
{
void HoldRepeatCommandMapping::executeCommandMapping(const RemoteMapState &currState)
{
    if (mappingSubset(currState) &&
        !(mapState.getNegKeysUsed() && negKeysSubset(mapState, currState)))
    {
        for (Command *cmd : mappedCommands)
        {
            if (!drivers->commandScheduler.isCommandScheduled(cmd))
            {
                drivers->commandScheduler.addCommand(cmd);
            }
        }
        commandsScheduled = true;
    }
    else
    {
        // While Commands may not be scheduled this prevents the unnecessary call of the
        // removeCommand function from the scheduler.
        if (commandsScheduled && endCommandsWhenNotHeld)
        {
            removeCommands();
            commandsScheduled = false;
        }
    }
}
}  // namespace control
}  // namespace tap
