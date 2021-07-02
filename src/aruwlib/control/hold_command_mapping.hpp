/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruwlib.
 *
 * aruwlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruwlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruwlib.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef HOLD_COMMAND_MAPPING_HPP_
#define HOLD_COMMAND_MAPPING_HPP_

#include "command_mapping.hpp"

namespace aruwlib
{
namespace control
{
class Command;
class RemoteMapState;

/**
 * A CommandMapping that adds `Command`s when the contained
 * mapping is a subset of the remote mapping and removes the `Command`s
 * when the mapping is no longer a subset.
 *
 * Additionally, When neg keys are being used and the mapping's neg keys
 * are a subset of the remote map state, the `Command`s are removed.
 */
class HoldCommandMapping : public CommandMapping
{
public:
    /**
     * Constructor must take the set of `Command`s and the RemoteMapState.
     */
    HoldCommandMapping(
        Drivers *drivers,
        const std::vector<Command *> cmds,
        const RemoteMapState &rms)
        : CommandMapping(drivers, cmds, rms),
          commandScheduled(false)
    {
    }

    /**
     * Default destructor.
     */
    ~HoldCommandMapping() override = default;

    void executeCommandMapping(const RemoteMapState &currState) override;

private:
    bool commandScheduled;
};  // class HoldCommandMapping
}  // namespace control
}  // namespace aruwlib

#endif  // HOLD_COMMAND_MAPPING_HPP_
