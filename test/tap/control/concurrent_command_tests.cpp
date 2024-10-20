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

#include "tap/control/concurrent_command.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/system_error.hpp"
#include "tap/mock/command_mock.hpp"

#include "test_command.hpp"
#include "test_subsystem.hpp"

using namespace tap::control;
using std::set;
using tap::Drivers;
using tap::mock::CommandMock;
using namespace testing;

static subsystem_scheduler_bitmap_t calcRequirementsBitwise(const set<Subsystem *> subRequirements)
{
    subsystem_scheduler_bitmap_t sum = 0;
    for (const auto sub : subRequirements)
    {
        sum |= (static_cast<subsystem_scheduler_bitmap_t>(1) << sub->getGlobalIdentifier());
    }
    return sum;
}

TEST(ConcurrentCommands, one_command_is_run)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    TestSubsystem s1(&drivers);
    scheduler.registerSubsystem(&s1);

    NiceMock<CommandMock> c1;

    set<Subsystem *> requirements = {&s1};
    EXPECT_CALL(c1, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));
    std::array<Command *, 1> commands = {&c1};
    ConcurrentCommand<1> command(commands, "test command");

    EXPECT_CALL(c1, isReady).WillOnce(Return(true));
    EXPECT_CALL(c1, initialize).Times(1);
    scheduler.addCommand(&command);

    EXPECT_CALL(c1, execute).Times(1);
    EXPECT_CALL(c1, isFinished).WillOnce(Return(true));
    EXPECT_CALL(c1, end(false)).Times(1);
    scheduler.run();

    EXPECT_FALSE(scheduler.isCommandScheduled(&command));
}

TEST(ConcurrentCommands, two_commands_are_run)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    TestSubsystem s1(&drivers);
    scheduler.registerSubsystem(&s1);
    NiceMock<CommandMock> c1;
    set<Subsystem *> requirements = {&s1};
    EXPECT_CALL(c1, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));

    TestSubsystem s2(&drivers);
    scheduler.registerSubsystem(&s2);
    NiceMock<CommandMock> c2;
    requirements = {&s2};
    EXPECT_CALL(c2, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));

    std::array<Command *, 2> commands = {&c1, &c2};
    ConcurrentCommand<2> command(commands, "test command");

    EXPECT_CALL(c1, isReady).WillOnce(Return(true));
    EXPECT_CALL(c1, initialize).Times(1);
    EXPECT_CALL(c2, isReady).WillOnce(Return(true));
    EXPECT_CALL(c2, initialize).Times(1);
    scheduler.addCommand(&command);

    EXPECT_CALL(c1, execute).Times(1);
    EXPECT_CALL(c1, isFinished).WillOnce(Return(true));
    EXPECT_CALL(c1, end(false)).Times(1);
    EXPECT_CALL(c2, execute).Times(1);
    EXPECT_CALL(c2, isFinished).WillOnce(Return(true));
    EXPECT_CALL(c2, end(false)).Times(1);
    scheduler.run();

    EXPECT_FALSE(scheduler.isCommandScheduled(&command));
}

TEST(ConcurrentCommands, two_commands_are_run_until_finished)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    TestSubsystem s1(&drivers);
    scheduler.registerSubsystem(&s1);
    NiceMock<CommandMock> c1;
    set<Subsystem *> requirements = {&s1};
    EXPECT_CALL(c1, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));

    TestSubsystem s2(&drivers);
    scheduler.registerSubsystem(&s2);
    NiceMock<CommandMock> c2;
    requirements = {&s2};
    EXPECT_CALL(c2, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));

    std::array<Command *, 2> commands = {&c1, &c2};
    ConcurrentCommand<2> command(commands, "test command");

    EXPECT_CALL(c1, isReady).WillOnce(Return(true));
    EXPECT_CALL(c1, initialize).Times(1);
    EXPECT_CALL(c2, isReady).WillOnce(Return(true));
    EXPECT_CALL(c2, initialize).Times(1);
    scheduler.addCommand(&command);

    EXPECT_CALL(c1, execute).Times(1);
    EXPECT_CALL(c1, isFinished).WillOnce(Return(true));
    EXPECT_CALL(c1, end(false)).Times(1);
    EXPECT_CALL(c2, execute).Times(2);
    EXPECT_CALL(c2, isFinished).WillOnce(Return(false)).WillOnce(Return(true));
    EXPECT_CALL(c2, end(false)).Times(1);

    scheduler.run();
    EXPECT_TRUE(scheduler.isCommandScheduled(&command));

    scheduler.run();
    EXPECT_FALSE(scheduler.isCommandScheduled(&command));
}

TEST(ConcurrentCommands, racing_two_commands_finishes_with_one)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    TestSubsystem s1(&drivers);
    scheduler.registerSubsystem(&s1);
    NiceMock<CommandMock> c1;
    set<Subsystem *> requirements = {&s1};
    EXPECT_CALL(c1, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));

    TestSubsystem s2(&drivers);
    scheduler.registerSubsystem(&s2);
    NiceMock<CommandMock> c2;
    requirements = {&s2};
    EXPECT_CALL(c2, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));

    std::array<Command *, 2> commands = {&c1, &c2};
    ConcurrentRaceCommand<2> command(commands, "test command");

    EXPECT_CALL(c1, isReady).WillOnce(Return(true));
    EXPECT_CALL(c1, initialize).Times(1);
    EXPECT_CALL(c2, isReady).WillOnce(Return(true));
    EXPECT_CALL(c2, initialize).Times(1);
    scheduler.addCommand(&command);

    EXPECT_CALL(c1, execute).Times(2);
    EXPECT_CALL(c1, isFinished).WillOnce(Return(false)).WillOnce(Return(true));
    EXPECT_CALL(c1, end(false)).Times(1);
    EXPECT_CALL(c2, execute).Times(2);
    EXPECT_CALL(c2, isFinished).Times(2).WillRepeatedly(Return(false));
    EXPECT_CALL(c2, end(true)).Times(1);

    scheduler.run();
    EXPECT_TRUE(scheduler.isCommandScheduled(&command));

    scheduler.run();
    EXPECT_FALSE(scheduler.isCommandScheduled(&command));
}

TEST(ConcurrentCommands, not_added_when_not_ready)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    TestSubsystem s1(&drivers);
    scheduler.registerSubsystem(&s1);

    NiceMock<CommandMock> c1;

    set<Subsystem *> requirements = {&s1};
    EXPECT_CALL(c1, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));
    std::array<Command *, 1> commands = {&c1};
    ConcurrentCommand<1> command(commands, "test command");

    EXPECT_CALL(c1, isReady).WillOnce(Return(false));
    scheduler.addCommand(&command);
    EXPECT_FALSE(scheduler.isCommandScheduled(&command));
}

TEST(ConcurrentCommands, cancelling_command_ends_internal_commands)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    TestSubsystem s1(&drivers);
    scheduler.registerSubsystem(&s1);

    NiceMock<CommandMock> c1;

    set<Subsystem *> requirements = {&s1};
    EXPECT_CALL(c1, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise(requirements)));
    std::array<Command *, 1> commands = {&c1};
    ConcurrentCommand<1> command(commands, "test command");

    EXPECT_CALL(c1, isReady).WillOnce(Return(true));
    EXPECT_CALL(c1, initialize).Times(1);
    scheduler.addCommand(&command);

    EXPECT_CALL(c1, execute).Times(1);
    EXPECT_CALL(c1, isFinished).WillOnce(Return(false));
    scheduler.run();
    EXPECT_TRUE(scheduler.isCommandScheduled(&command));

    EXPECT_CALL(c1, end(true)).Times(1);
    scheduler.removeCommand(&command, true);
    EXPECT_FALSE(scheduler.isCommandScheduled(&command));
}

TEST(ConcurrentCommands, null_command_asserts)
{
    std::array<Command *, 1> commands = {nullptr};
    ASSERT_DEATH({ ConcurrentCommand<1> command(commands, "test command"); }, ".*");
}

TEST(ConcurrentCommands, overlapping_requirements_asserts)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    TestSubsystem s1(&drivers);
    scheduler.registerSubsystem(&s1);
    TestCommand c1(&s1);
    TestCommand c2(&s1);

    std::array<Command *, 2> commands = {&c1, &c2};
    ASSERT_DEATH({ ConcurrentCommand<2> command(commands, "test command"); }, ".*");
}
