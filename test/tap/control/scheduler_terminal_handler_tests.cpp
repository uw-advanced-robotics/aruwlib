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

#include <gtest/gtest.h>

#include "tap/control/scheduler_terminal_handler.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/command_mock.hpp"
#include "tap/mock/subsystem_mock.hpp"
#include "tap/stub/terminal_device_stub.hpp"

using namespace tap::control;
using namespace testing;
using namespace tap;
using namespace tap::mock;
using namespace testing;

TEST(SchedulerTerminalHandler, init__adds_to_terminal_handler)
{
    Drivers drivers;
    SchedulerTerminalHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    EXPECT_CALL(drivers.terminalSerial, addHeader(_, &serialHandler));

    serialHandler.init();
}

TEST(SchedulerTerminalHandler, terminalSerialCallback__doesnt_crash_if_input_blank)
{
    Drivers drivers;
    SchedulerTerminalHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char input[] = "";
    serialHandler.terminalSerialCallback(input, stream, false);

    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), HasSubstr("Usage"));
}

TEST(SchedulerTerminalHandler, terminalSerialCallback__prints_usage_if_invalid_input)
{
    Drivers drivers;
    SchedulerTerminalHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char input[] = "\t \r sdsdf";
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input, stream, false));

    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), HasSubstr("Usage"));
}

TEST(SchedulerTerminalHandler, terminalSerialCallback__prints_usage_if_help_specified)
{
    Drivers drivers;
    SchedulerTerminalHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char input[] = "-H";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    EXPECT_THAT(terminalDevice.readAllItemsFromWriteBufferToString(), HasSubstr("Usage"));
}

TEST(
    SchedulerTerminalHandler,
    terminalSerialCallback__prints_no_subsystems_or_commands_when_none_in_scheduler)
{
    Drivers drivers;
    SchedulerTerminalHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    ON_CALL(drivers.commandScheduler, cmdMapBegin)
        .WillByDefault([&]() { return drivers.commandScheduler.CommandScheduler::cmdMapBegin(); });
    ON_CALL(drivers.commandScheduler, cmdMapEnd)
        .WillByDefault([&]() { return drivers.commandScheduler.CommandScheduler::cmdMapEnd(); });
    ON_CALL(drivers.commandScheduler, subMapBegin)
        .WillByDefault([&]() { return drivers.commandScheduler.CommandScheduler::subMapBegin(); });
    ON_CALL(drivers.commandScheduler, subMapEnd)
        .WillByDefault([&]() { return drivers.commandScheduler.CommandScheduler::subMapEnd(); });

    char input[] = "allsubcmd";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();

    EXPECT_THAT(output, HasSubstr("Subsystems:\nCommands:\n"));
}

TEST(SchedulerTerminalHandler, terminalSerialCallback__prints_subsystems_commands)
{
    Drivers drivers;
    SchedulerTerminalHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    NiceMock<SubsystemMock> s1(&drivers);
    NiceMock<SubsystemMock> s2(&drivers);
    NiceMock<CommandMock> c1;
    NiceMock<CommandMock> c2;

    ON_CALL(drivers.commandScheduler, cmdMapBegin)
        .WillByDefault([&]() { return drivers.commandScheduler.CommandScheduler::cmdMapBegin(); });
    ON_CALL(drivers.commandScheduler, cmdMapEnd)
        .WillByDefault([&]() { return drivers.commandScheduler.CommandScheduler::cmdMapEnd(); });
    ON_CALL(drivers.commandScheduler, subMapBegin)
        .WillByDefault([&]() { return drivers.commandScheduler.CommandScheduler::subMapBegin(); });
    ON_CALL(drivers.commandScheduler, subMapEnd)
        .WillByDefault([&]() { return drivers.commandScheduler.CommandScheduler::subMapEnd(); });

    ON_CALL(c1, getName).WillByDefault(Return("c1"));
    ON_CALL(c2, getName).WillByDefault(Return("c2"));
    ON_CALL(s1, getName).WillByDefault(Return("s1"));
    ON_CALL(s2, getName).WillByDefault(Return("s2"));

    ON_CALL(c1, getRequirementsBitwise).WillByDefault(Return(1 << s1.getGlobalIdentifier()));
    ON_CALL(c2, getRequirementsBitwise).WillByDefault(Return(1 << s2.getGlobalIdentifier()));

    drivers.commandScheduler.CommandScheduler::registerSubsystem(&s1);
    drivers.commandScheduler.CommandScheduler::registerSubsystem(&s2);
    drivers.commandScheduler.CommandScheduler::addCommand(&c1);
    drivers.commandScheduler.CommandScheduler::addCommand(&c2);

    char input[] = "allsubcmd";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("c1"));
    EXPECT_THAT(output, HasSubstr("c2"));
    EXPECT_THAT(output, HasSubstr("s1"));
    EXPECT_THAT(output, HasSubstr("s2"));
}

TEST(
    SchedulerTerminalHandler,
    terminalSerialCallback__returns_false_if_streaming_enabled_and_help_specified)
{
    Drivers drivers;
    SchedulerTerminalHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    char input[] = "-H";
    EXPECT_FALSE(serialHandler.terminalSerialCallback(input, stream, true));
}

TEST(SchedulerTerminalHandler, terminalSerialStreamCallback__repeatedly_prints_commands_and_subs)
{
    Drivers drivers;
    SchedulerTerminalHandler serialHandler(&drivers);
    tap::stub::TerminalDeviceStub terminalDevice(&drivers);
    modm::IOStream stream(terminalDevice);

    NiceMock<SubsystemMock> s1(&drivers);
    NiceMock<SubsystemMock> s2(&drivers);
    NiceMock<CommandMock> c1;
    NiceMock<CommandMock> c2;

    ON_CALL(drivers.commandScheduler, cmdMapBegin)
        .WillByDefault([&]() { return drivers.commandScheduler.CommandScheduler::cmdMapBegin(); });
    ON_CALL(drivers.commandScheduler, cmdMapEnd)
        .WillByDefault([&]() { return drivers.commandScheduler.CommandScheduler::cmdMapEnd(); });
    ON_CALL(drivers.commandScheduler, subMapBegin)
        .WillByDefault([&]() { return drivers.commandScheduler.CommandScheduler::subMapBegin(); });
    ON_CALL(drivers.commandScheduler, subMapEnd)
        .WillByDefault([&]() { return drivers.commandScheduler.CommandScheduler::subMapEnd(); });

    ON_CALL(c1, getName).WillByDefault(Return("c1"));
    ON_CALL(c2, getName).WillByDefault(Return("c2"));
    ON_CALL(s1, getName).WillByDefault(Return("s1"));
    ON_CALL(s2, getName).WillByDefault(Return("s2"));

    ON_CALL(c1, getRequirementsBitwise).WillByDefault(Return(1 << s1.getGlobalIdentifier()));
    ON_CALL(c2, getRequirementsBitwise).WillByDefault(Return(1 << s2.getGlobalIdentifier()));

    drivers.commandScheduler.CommandScheduler::registerSubsystem(&s1);
    drivers.commandScheduler.CommandScheduler::addCommand(&c1);

    char input[] = "allsubcmd";
    EXPECT_TRUE(serialHandler.terminalSerialCallback(input, stream, false));

    std::string output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("c1"));
    EXPECT_THAT(output, HasSubstr("s1"));

    serialHandler.terminalSerialStreamCallback(stream);

    output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("c1"));
    EXPECT_THAT(output, HasSubstr("s1"));

    drivers.commandScheduler.CommandScheduler::registerSubsystem(&s2);
    drivers.commandScheduler.CommandScheduler::addCommand(&c2);

    serialHandler.terminalSerialStreamCallback(stream);

    output = terminalDevice.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("c1"));
    EXPECT_THAT(output, HasSubstr("c2"));
    EXPECT_THAT(output, HasSubstr("s1"));
    EXPECT_THAT(output, HasSubstr("s2"));
}
