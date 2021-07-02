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

#include "command_mapper_format_generator.hpp"

#include "command.hpp"
#include "command_mapper.hpp"
#include "command_mapping.hpp"
#include "remote_map_state.hpp"

namespace aruwlib
{
namespace control
{
const std::vector<std::string> CommandMapperFormatGenerator::generateMappings() const
{
    std::vector<std::string> out;

    for (std::size_t i = 0; i < mapper.getSize(); i++)
    {
        const CommandMapping *mapping = mapper.getAtIndex(i);
        out.push_back(
            formattedRemoteMapState(mapping->getAssociatedRemoteMapState()) + ":\t" +
            formattedMappedCommands(mapping->getAssociatedCommands()));
    }
    return out;
}

const std::string CommandMapperFormatGenerator::formattedRemoteMapState(
    const RemoteMapState &ms) const
{
    std::string out = "[";
    if (ms.getKeys() != 0)
    {
        out += "keys: ";
        out += keyMapToString(ms.getKeys());
        out += ", ";
    }
    if (ms.getNegKeys() != 0)
    {
        out += "neg keys: ";
        out += keyMapToString(ms.getNegKeys());
        out += ", ";
    }
    if (ms.getLMouseButton())
    {
        out += "left mouse pressed, ";
    }
    if (ms.getRMouseButton())
    {
        out += "right mouse pressed, ";
    }
    if (ms.getLSwitch() != Remote::SwitchState::UNKNOWN)
    {
        out += "left switch: ";
        out += switchStateToString(ms.getLSwitch());
        out += ", ";
    }
    if (ms.getRSwitch() != Remote::SwitchState::UNKNOWN)
    {
        out += "right switch: ";
        out += switchStateToString(ms.getRSwitch());
        out += ", ";
    }
    out = out.length() == 1 ? "[none" : out.substr(0, out.length() - 2);
    return out + "]";
}

const std::string CommandMapperFormatGenerator::formattedMappedCommands(
    const std::vector<Command *> mc) const
{
    if (mc.size() == 0)
    {
        return "[none]";
    }
    std::string out = "[";
    out += mc[0]->getName();
    for (std::size_t i = 1; i < mc.size(); i++)
    {
        out += ", ";
        out += mc[i]->getName();
    }
    return out + "]";
}

constexpr std::string_view CommandMapperFormatGenerator::switchStateToString(
    Remote::SwitchState state) const
{
    switch (state)
    {
        case Remote::SwitchState::DOWN:
            return "down";
        case Remote::SwitchState::MID:
            return "mid";
        case Remote::SwitchState::UNKNOWN:
            return "unknown";
        case Remote::SwitchState::UP:
            return "up";
        default:
            return "";
    }
}

const std::string CommandMapperFormatGenerator::keyMapToString(uint16_t keys) const
{
    if (keys == 0)
    {
        return "{ NONE }";
    }
    std::string out;
    out = "{";
    if (keys & (1 << static_cast<int>(Remote::Key::A)))
    {
        out += "A, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::B)))
    {
        out += "B, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::C)))
    {
        out += "C, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::D)))
    {
        out += "D, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::E)))
    {
        out += "E, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::F)))
    {
        out += "F, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::G)))
    {
        out += "G, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::Q)))
    {
        out += "Q, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::R)))
    {
        out += "R, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::S)))
    {
        out += "S, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::V)))
    {
        out += "V, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::W)))
    {
        out += "W, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::X)))
    {
        out += "X, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::Z)))
    {
        out += "Z, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::SHIFT)))
    {
        out += "SHIFT, ";
    }
    if (keys & (1 << static_cast<int>(Remote::Key::CTRL)))
    {
        out += "CTRL, ";
    }
    return out.substr(0, out.length() - 2) + "}";
}
}  // namespace control
}  // namespace aruwlib
