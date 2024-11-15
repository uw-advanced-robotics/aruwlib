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

#include "tap/architecture/clock.hpp"
#include "tap/control/command.hpp"
#include "tap/control/command_scheduler.hpp"
#include "tap/control/comprised_command.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/command_mock.hpp"
#include "tap/mock/remote_mock.hpp"
#include "tap/mock/subsystem_mock.hpp"

using std::set;
using tap::Drivers;
using tap::mock::CommandMock;
using tap::mock::RemoteMock;
using tap::mock::SubsystemMock;
using namespace tap::control;
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

class RemoteSafeDisconnectFunction : public tap::control::SafeDisconnectFunction
{
public:
    RemoteSafeDisconnectFunction(Drivers *drivers) { this->drivers = drivers; };
    virtual bool operator()() { return !drivers->remote.isConnected(); }

private:
    Drivers *drivers;
};

TEST(CommandScheduler, constructor_multiple_master_schedulers_throws_error)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(2);

    CommandScheduler scheduler1(&drivers, true);
    CommandScheduler scheduler2(&drivers, true);
}

TEST(CommandScheduler, constructor_single_master_schedulers_does_not_throw_error)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    CommandScheduler scheduler1(&drivers);
    CommandScheduler scheduler2(&drivers);
}

TEST(CommandScheduler, registerSubsystem_single_subsystem_added_and_ran)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    SubsystemMock sub(&drivers);

    EXPECT_CALL(sub, refresh);
    EXPECT_CALL(sub, getDefaultCommand).WillOnce(Return(nullptr));

    scheduler.registerSubsystem(&sub);
    scheduler.run();
}

TEST(CommandScheduler, registerSubsystem_multiple_unique_subsystems_added_and_ran)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    SubsystemMock sub1(&drivers);
    SubsystemMock sub2(&drivers);
    SubsystemMock sub3(&drivers);

    EXPECT_CALL(sub1, refresh);
    EXPECT_CALL(sub1, getDefaultCommand).WillOnce(Return(nullptr));
    EXPECT_CALL(sub2, refresh);
    EXPECT_CALL(sub2, getDefaultCommand).WillOnce(Return(nullptr));
    EXPECT_CALL(sub3, refresh);
    EXPECT_CALL(sub3, getDefaultCommand).WillOnce(Return(nullptr));

    scheduler.registerSubsystem(&sub1);
    scheduler.registerSubsystem(&sub2);
    scheduler.registerSubsystem(&sub3);
    scheduler.run();
}

TEST(CommandScheduler, registerSubsystem_single_subsystem_added_multiple_times_only_added_once)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    SubsystemMock sub(&drivers);

    EXPECT_CALL(sub, refresh);
    EXPECT_CALL(sub, getDefaultCommand).WillOnce(Return(nullptr));
    EXPECT_CALL(drivers.errorController, addToErrorList);

    scheduler.registerSubsystem(&sub);
    scheduler.registerSubsystem(&sub);
    scheduler.run();
}

TEST(CommandScheduler, registerSubsystem_doesnt_register_nullptr_subsystem)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    EXPECT_CALL(drivers.errorController, addToErrorList);

    scheduler.registerSubsystem(nullptr);
    EXPECT_FALSE(scheduler.isSubsystemRegistered(nullptr));
}

TEST(CommandScheduler, registerSubsystem_big_batch_subsystem_assertion_succeeds)
{
    constexpr int SUBS_TO_REGISTER =
        std::min(sizeof(command_scheduler_bitmap_t), sizeof(subsystem_scheduler_bitmap_t)) * 8;
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock *subsystemArrToRegister[SUBS_TO_REGISTER / 2];
    SubsystemMock *subsystemArrNotToRegister[SUBS_TO_REGISTER / 2];

    for (int i = 0; i < SUBS_TO_REGISTER / 2; i++)
    {
        subsystemArrToRegister[i] = new SubsystemMock(&drivers);
        subsystemArrNotToRegister[i] = new SubsystemMock(&drivers);
        scheduler.registerSubsystem(subsystemArrToRegister[i]);
    }

    for (int i = 0; i < SUBS_TO_REGISTER / 2; i++)
    {
        EXPECT_TRUE(scheduler.isSubsystemRegistered(subsystemArrToRegister[i]));
        EXPECT_FALSE(scheduler.isSubsystemRegistered(subsystemArrNotToRegister[i]));
    }

    for (int i = 0; i < SUBS_TO_REGISTER / 2; i++)
    {
        delete subsystemArrToRegister[i];
        delete subsystemArrNotToRegister[i];
    }
}

TEST(CommandScheduler, isSubsystemRegistered_returns_true_if_registered)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    SubsystemMock sub1(&drivers);
    SubsystemMock sub2(&drivers);
    SubsystemMock sub3(&drivers);

    scheduler.registerSubsystem(&sub1);
    scheduler.registerSubsystem(&sub2);
    scheduler.registerSubsystem(&sub3);
    EXPECT_TRUE(scheduler.isSubsystemRegistered(&sub1));
    EXPECT_TRUE(scheduler.isSubsystemRegistered(&sub2));
    EXPECT_TRUE(scheduler.isSubsystemRegistered(&sub3));
}

TEST(CommandScheduler, isSubsystemRegistered_returns_false_if_not_registered)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    SubsystemMock sub1(&drivers);
    SubsystemMock sub2(&drivers);
    SubsystemMock sub3(&drivers);

    scheduler.registerSubsystem(&sub1);
    EXPECT_FALSE(scheduler.isSubsystemRegistered(&sub2));
    EXPECT_FALSE(scheduler.isSubsystemRegistered(&sub3));
}

TEST(CommandScheduler, addCommand_null_added_command_raises_error)
{
    // Setup.
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    // Expect an error when the command is null.
    EXPECT_CALL(drivers.errorController, addToErrorList);
    scheduler.addCommand(nullptr);
}

TEST(CommandScheduler, addCommand_with_no_subsystem_registered_raises_error)
{
    // Setup, create a subsystem but dont add it to command scheduler.
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    SubsystemMock s(&drivers);
    NiceMock<CommandMock> c;
    set<Subsystem *> subRequirements{&s};

    // Expect an error with no subsystem in the command scheduler.
    EXPECT_CALL(drivers.errorController, addToErrorList);
    EXPECT_CALL(c, getRequirementsBitwise)
        .WillOnce(Return(calcRequirementsBitwise(subRequirements)));

    scheduler.addCommand(&c);
}

TEST(CommandScheduler, addCommand_with_not_all_subsystems_registered_raises_error)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    SubsystemMock s1(&drivers);
    SubsystemMock s2(&drivers);
    SubsystemMock s3(&drivers);
    NiceMock<CommandMock> c;
    set<Subsystem *> subsystemRequirements{&s1, &s2, &s3};

    EXPECT_CALL(c, getRequirementsBitwise)
        .Times(3)
        .WillRepeatedly(Return(calcRequirementsBitwise(subsystemRequirements)));
    EXPECT_CALL(drivers.errorController, addToErrorList).Times(3);

    // No subsystems added, will fail.
    scheduler.addCommand(&c);

    // One subsystem registered, will fail.
    scheduler.registerSubsystem(&s1);
    scheduler.addCommand(&c);

    // Two subsystems registered, will fail.
    scheduler.registerSubsystem(&s2);
    scheduler.addCommand(&c);
}

TEST(CommandScheduler, isCommandScheduled_nullptr_command_returns_false)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    EXPECT_FALSE(scheduler.isCommandScheduled(nullptr));
}

TEST(CommandScheduler, isSubsystemRegistered_nullptr_subsystem_returns_false)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    EXPECT_FALSE(scheduler.isSubsystemRegistered(nullptr));
}

TEST(
    CommandScheduler,
    addCommand_and_isCommandScheduled_command_successfully_added_with_single_dependent_subsystem)
{
    // Setup, create a subsystem but dont add it to command scheduler.
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    SubsystemMock s(&drivers);
    NiceMock<CommandMock> c;

    // Set up another command that we can set an expectation on the `initialize` function.
    set<Subsystem *> cmdMockRequirements{&s};
    EXPECT_CALL(c, getRequirementsBitwise)
        .WillOnce(Return(calcRequirementsBitwise(cmdMockRequirements)));
    EXPECT_CALL(c, initialize);

    scheduler.registerSubsystem(&s);

    // Expect no errors, and the method initalize is called on the command.
    scheduler.addCommand(&c);
    EXPECT_TRUE(scheduler.isCommandScheduled(&c));
}

TEST(
    CommandScheduler,
    addCommand_and_isCommandScheduled_command_added_successfully_with_multiple_subsystem_dependencies_all_registered)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    SubsystemMock s1(&drivers);
    SubsystemMock s2(&drivers);
    SubsystemMock s3(&drivers);
    NiceMock<CommandMock> c;
    set<Subsystem *> subsystemRequirements{&s1, &s2, &s3};

    EXPECT_CALL(c, getRequirementsBitwise)
        .WillOnce(Return(calcRequirementsBitwise(subsystemRequirements)));
    EXPECT_CALL(c, initialize);

    // Add all subsystems, registering command will succeed
    scheduler.registerSubsystem(&s1);
    scheduler.registerSubsystem(&s2);
    scheduler.registerSubsystem(&s3);
    scheduler.addCommand(&c);
    EXPECT_TRUE(scheduler.isCommandScheduled(&c));
}

TEST(CommandScheduler, addCommand_successfully_removes_previously_added_command_with_same_subsystem)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    SubsystemMock s(&drivers);
    NiceMock<CommandMock> c1;
    NiceMock<CommandMock> c2;

    set<Subsystem *> cmdMockRequirement{&s};
    EXPECT_CALL(c1, getRequirementsBitwise)
        .Times(3)
        .WillRepeatedly(Return(calcRequirementsBitwise(cmdMockRequirement)));
    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(c1, end);
    EXPECT_CALL(c2, getRequirementsBitwise)
        .WillOnce(Return(calcRequirementsBitwise(cmdMockRequirement)));
    EXPECT_CALL(c2, initialize);

    // Set up the test, adding the first command that will later be removed.
    scheduler.registerSubsystem(&s);
    scheduler.addCommand(&c1);
    scheduler.addCommand(&c2);

    EXPECT_CALL(s, refresh);
    EXPECT_CALL(c2, execute);
    EXPECT_CALL(c2, isFinished).WillOnce(Return(false));

    scheduler.run();
}

TEST(
    CommandScheduler,
    addCommand_successfully_removes_previously_added_command_with_subset_of_subsystem_requirements)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock s1(&drivers);
    SubsystemMock s2(&drivers);
    SubsystemMock s3(&drivers);

    NiceMock<CommandMock> c1;
    NiceMock<CommandMock> c2;

    set<Subsystem *> subRequirementsC1{&s1, &s2};
    set<Subsystem *> subRequirementsC2{&s1, &s2, &s3};

    EXPECT_CALL(c1, getRequirementsBitwise)
        .Times(3)
        .WillRepeatedly(Return(calcRequirementsBitwise(subRequirementsC1)));
    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(c1, end);
    EXPECT_CALL(c2, getRequirementsBitwise)
        .WillOnce(Return(calcRequirementsBitwise(subRequirementsC2)));
    EXPECT_CALL(c2, initialize);

    scheduler.registerSubsystem(&s1);
    scheduler.registerSubsystem(&s2);
    scheduler.registerSubsystem(&s3);

    // The first command is associated with the first two subsystems.
    scheduler.addCommand(&c1);
    // The second command is associated with all three subsystems.
    scheduler.addCommand(&c2);

    EXPECT_CALL(s1, refresh);
    EXPECT_CALL(s2, refresh);
    EXPECT_CALL(s3, refresh);
    EXPECT_CALL(c2, execute);
    EXPECT_CALL(c2, isFinished).WillOnce(Return(false));

    scheduler.run();
}

TEST(
    CommandScheduler,
    addCommand_multiple_successfully_removes_previously_added_command_if_single_shared_subsystem_requirement)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock s1(&drivers);
    SubsystemMock s2(&drivers);
    SubsystemMock s3(&drivers);

    NiceMock<CommandMock> c1;
    NiceMock<CommandMock> c2;

    set<Subsystem *> subRequirementsC1{&s1, &s2};
    set<Subsystem *> subRequirementsC2{&s2, &s3};

    EXPECT_CALL(c1, getRequirementsBitwise)
        .Times(3)
        .WillRepeatedly(Return(calcRequirementsBitwise(subRequirementsC1)));
    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(c1, end);
    EXPECT_CALL(c2, getRequirementsBitwise)
        .WillOnce(Return(calcRequirementsBitwise(subRequirementsC2)));
    EXPECT_CALL(c2, initialize);

    scheduler.registerSubsystem(&s1);
    scheduler.registerSubsystem(&s2);
    scheduler.registerSubsystem(&s3);

    // The first command is associated with the first two subsystems.
    scheduler.addCommand(&c1);
    // The second command is associated with last two subsystems.
    scheduler.addCommand(&c2);

    EXPECT_CALL(s1, refresh);
    EXPECT_CALL(s2, refresh);
    EXPECT_CALL(s3, refresh);
    EXPECT_CALL(c2, execute);
    EXPECT_CALL(c2, isFinished).WillOnce(Return(false));
    EXPECT_CALL(s1, getDefaultCommand);

    scheduler.run();
}

TEST(
    CommandScheduler,
    addCommand_add_two_commands_with_overlapping_sub_requirements_but_second_addCommand_fails_so_first_still_remains_added)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock s1(&drivers);
    SubsystemMock s2(&drivers);
    SubsystemMock s3(&drivers);

    NiceMock<CommandMock> c1;
    NiceMock<CommandMock> c2;
    NiceMock<CommandMock> c3;

    set<Subsystem *> subRequirementsC1{&s1, &s2};
    set<Subsystem *> subRequirementsC2{&s2, &s3};
    set<Subsystem *> subRequirementsC3{&s1};

    EXPECT_CALL(c1, getRequirementsBitwise)
        .Times(3)
        .WillRepeatedly(Return(calcRequirementsBitwise(subRequirementsC1)));
    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(c1, end);
    EXPECT_CALL(c2, getRequirementsBitwise)
        .WillOnce(Return(calcRequirementsBitwise(subRequirementsC2)));
    EXPECT_CALL(c3, getRequirementsBitwise)
        .WillOnce(Return(calcRequirementsBitwise(subRequirementsC3)));
    EXPECT_CALL(c3, initialize);
    EXPECT_CALL(drivers.errorController, addToErrorList);

    scheduler.registerSubsystem(&s1);
    scheduler.registerSubsystem(&s2);

    // The first command is associated with the first two subsystems.
    scheduler.addCommand(&c1);
    // The second command is associated with last two subsystems. This should fail.
    scheduler.addCommand(&c2);
    EXPECT_FALSE(scheduler.isCommandScheduled(&c2));
    scheduler.addCommand(&c3);
    EXPECT_FALSE(scheduler.isCommandScheduled(&c2));

    EXPECT_CALL(s1, refresh);
    EXPECT_CALL(s2, refresh);
    EXPECT_CALL(s2, getDefaultCommand);
    EXPECT_CALL(c3, execute);
    EXPECT_CALL(c3, isFinished).WillOnce(Return(false));

    scheduler.run();
}

TEST(
    CommandScheduler,
    addCommand_multiple_successfully_removes_previously_added_command_with_superset_of_subsystems)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock s1(&drivers);
    SubsystemMock s2(&drivers);
    SubsystemMock s3(&drivers);

    NiceMock<CommandMock> c1;
    NiceMock<CommandMock> c2;

    set<Subsystem *> subRequirementsC1{&s1, &s2, &s3};
    set<Subsystem *> subRequirementsC2{&s1, &s2};

    EXPECT_CALL(c1, getRequirementsBitwise)
        .Times(3)
        .WillRepeatedly(Return(calcRequirementsBitwise(subRequirementsC1)));
    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(c1, end);
    EXPECT_CALL(c2, getRequirementsBitwise)
        .WillOnce(Return(calcRequirementsBitwise(subRequirementsC2)));
    EXPECT_CALL(c2, initialize);

    scheduler.registerSubsystem(&s1);
    scheduler.registerSubsystem(&s2);
    scheduler.registerSubsystem(&s3);

    // The first command is associated with all three subsystems.
    scheduler.addCommand(&c1);
    // When the second command is added, the first command is removed completely from the
    scheduler.addCommand(&c2);

    EXPECT_CALL(s1, refresh);
    EXPECT_CALL(s2, refresh);
    EXPECT_CALL(s3, refresh);
    EXPECT_CALL(c2, execute);
    EXPECT_CALL(c2, isFinished).WillOnce(Return(false));
    // The third subsystem doesn't have a command associated with it.
    EXPECT_CALL(s3, getDefaultCommand).WillOnce(Return(nullptr));

    scheduler.run();
}

TEST(CommandScheduler, addCommand_add_multiple_commands_with_overlapping_sub_requirements)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock s1(&drivers);
    SubsystemMock s2(&drivers);
    SubsystemMock s3(&drivers);
    SubsystemMock s4(&drivers);
    SubsystemMock s5(&drivers);

    NiceMock<CommandMock> c1;
    NiceMock<CommandMock> c2;
    NiceMock<CommandMock> c3;
    NiceMock<CommandMock> c4;

    set<Subsystem *> subRequirementsC1{&s1, &s2, &s3};
    set<Subsystem *> subRequirementsC2{&s1, &s2};
    set<Subsystem *> subRequirementsC3{&s3, &s4};
    set<Subsystem *> subRequirementsC4{&s2, &s3, &s5};

    EXPECT_CALL(c1, getRequirementsBitwise)
        .Times(3)
        .WillRepeatedly(Return(calcRequirementsBitwise(subRequirementsC1)));
    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(c1, end);
    EXPECT_CALL(c2, getRequirementsBitwise)
        .Times(4)
        .WillRepeatedly(Return(calcRequirementsBitwise(subRequirementsC2)));
    EXPECT_CALL(c2, initialize);
    EXPECT_CALL(c2, end);
    EXPECT_CALL(c3, getRequirementsBitwise)
        .Times(3)
        .WillRepeatedly(Return(calcRequirementsBitwise(subRequirementsC3)));
    EXPECT_CALL(c3, initialize);
    EXPECT_CALL(c3, end);
    EXPECT_CALL(c4, getRequirementsBitwise)
        .WillOnce(Return(calcRequirementsBitwise(subRequirementsC4)));
    EXPECT_CALL(c4, initialize);

    scheduler.registerSubsystem(&s1);
    scheduler.registerSubsystem(&s2);
    scheduler.registerSubsystem(&s3);
    scheduler.registerSubsystem(&s4);
    scheduler.registerSubsystem(&s5);

    scheduler.addCommand(&c1);
    scheduler.addCommand(&c2);
    scheduler.addCommand(&c3);

    EXPECT_TRUE(scheduler.isCommandScheduled(&c2));
    EXPECT_TRUE(scheduler.isCommandScheduled(&c3));

    scheduler.addCommand(&c4);

    EXPECT_CALL(s1, refresh);
    EXPECT_CALL(s2, refresh);
    EXPECT_CALL(s3, refresh);
    EXPECT_CALL(s4, refresh);
    EXPECT_CALL(s5, refresh);
    EXPECT_CALL(s1, getDefaultCommand);
    EXPECT_CALL(s4, getDefaultCommand);
    EXPECT_CALL(c4, execute);
    EXPECT_CALL(c4, isFinished).WillOnce(Return(false));

    scheduler.run();
}

TEST(CommandScheduler, runHardwareTest_does_not_segfault_with_nullptr_command)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock s1(&drivers);
    EXPECT_CALL(s1, getTestCommand).WillRepeatedly(Return(nullptr));
    EXPECT_CALL(s1, refresh);
    EXPECT_CALL(s1, getDefaultCommand).WillOnce(Return(nullptr));

    scheduler.registerSubsystem(&s1);
    scheduler.runHardwareTest(&s1);
    scheduler.run();
    EXPECT_FALSE(scheduler.hasPassedTest(&s1));
}

TEST(CommandScheduler, runHardwareTest_runs_test_command)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock s1(&drivers);
    set<Subsystem *> c1Requirements = {&s1};
    NiceMock<CommandMock> c1;
    EXPECT_CALL(c1, getRequirementsBitwise)
        .Times(2)  // Called when adding and removing the command
        .WillRepeatedly(Return(calcRequirementsBitwise(c1Requirements)));
    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(s1, getTestCommand).WillRepeatedly(Return(&c1));
    EXPECT_CALL(s1, refresh);
    EXPECT_CALL(s1, getDefaultCommand).WillRepeatedly(Return(nullptr));

    // Called once in the run loop.
    EXPECT_CALL(c1, execute).Times(1);
    EXPECT_CALL(c1, isFinished).Times(2).WillRepeatedly(Return(true));
    EXPECT_CALL(c1, end(false)).Times(1);

    scheduler.registerSubsystem(&s1);
    scheduler.runHardwareTest(&s1);
    EXPECT_TRUE(scheduler.isRunningTest(&s1));
    EXPECT_EQ(scheduler.countRunningHardwareTests(), 1);
    scheduler.run();
    EXPECT_TRUE(scheduler.hasPassedTest(&s1));
}

TEST(CommandScheduler, runHardwareTest_then_stop_initializes_then_ends_test_command)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock s1(&drivers);
    set<Subsystem *> c1Requirements = {&s1};
    NiceMock<CommandMock> c1;
    EXPECT_CALL(c1, getRequirementsBitwise)
        .Times(2)  // Called when adding and removing the command
        .WillRepeatedly(Return(calcRequirementsBitwise(c1Requirements)));
    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(s1, getTestCommand).WillRepeatedly(Return(&c1));
    EXPECT_CALL(s1, refresh);
    EXPECT_CALL(s1, getDefaultCommand).WillRepeatedly(Return(nullptr));

    // Not called in the run loop.
    EXPECT_CALL(c1, execute).Times(0);
    EXPECT_CALL(c1, end(true)).Times(1);

    scheduler.registerSubsystem(&s1);
    scheduler.runHardwareTest(&s1);
    scheduler.stopHardwareTest(&s1);
    scheduler.run();
    EXPECT_FALSE(scheduler.hasPassedTest(&s1));
}

TEST(CommandScheduler, runAllHardwareTests_runs_all_valid_test_commands)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock s1(&drivers);
    set<Subsystem *> c1Requirements = {&s1};
    NiceMock<CommandMock> c1;
    EXPECT_CALL(c1, getRequirementsBitwise)
        .Times(2)  // Called when adding and removing the command
        .WillRepeatedly(Return(calcRequirementsBitwise(c1Requirements)));
    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(s1, getTestCommand).WillRepeatedly(Return(&c1));
    EXPECT_CALL(s1, refresh);
    EXPECT_CALL(s1, getDefaultCommand).WillRepeatedly(Return(nullptr));

    // Called once in the run loop.
    EXPECT_CALL(c1, execute).Times(1);
    EXPECT_CALL(c1, isFinished).Times(2).WillRepeatedly(Return(true));
    EXPECT_CALL(c1, end(false)).Times(1);

    SubsystemMock s2(&drivers);
    EXPECT_CALL(s2, getTestCommand).WillRepeatedly(Return(nullptr));
    EXPECT_CALL(s2, refresh);
    EXPECT_CALL(s2, getDefaultCommand).WillRepeatedly(Return(nullptr));

    scheduler.registerSubsystem(&s1);
    scheduler.registerSubsystem(&s2);
    scheduler.runAllHardwareTests();
    scheduler.run();
    EXPECT_TRUE(scheduler.hasPassedTest(&s1));
    EXPECT_FALSE(scheduler.hasPassedTest(&s2));
}

TEST(CommandScheduler, stopAllHardwareTests_stops_all_valid_test_commands)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock s1(&drivers);
    set<Subsystem *> c1Requirements = {&s1};
    NiceMock<CommandMock> c1;
    EXPECT_CALL(c1, getRequirementsBitwise)
        .Times(2)  // Called when adding and removing the command
        .WillRepeatedly(Return(calcRequirementsBitwise(c1Requirements)));
    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(s1, getTestCommand).WillRepeatedly(Return(&c1));
    EXPECT_CALL(s1, refresh);
    EXPECT_CALL(s1, getDefaultCommand).WillRepeatedly(Return(nullptr));

    // Not called in the run loop.
    EXPECT_CALL(c1, execute).Times(0);
    EXPECT_CALL(c1, end(true)).Times(1);

    SubsystemMock s2(&drivers);
    EXPECT_CALL(s2, getTestCommand).WillRepeatedly(Return(nullptr));
    EXPECT_CALL(s2, refresh);
    EXPECT_CALL(s2, getDefaultCommand).WillRepeatedly(Return(nullptr));

    scheduler.registerSubsystem(&s1);
    scheduler.registerSubsystem(&s2);
    scheduler.runAllHardwareTests();
    scheduler.stopAllHardwareTests();
    scheduler.run();
    EXPECT_FALSE(scheduler.hasPassedTest(&s1));
}

TEST(CommandScheduler, runHardwareTest_cancels_scheduled_command_and_then_runs_test_command)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock s1(&drivers);
    EXPECT_CALL(s1, refresh).Times(3);

    set<Subsystem *> requirements = {&s1};
    NiceMock<CommandMock> c1;
    EXPECT_CALL(c1, getRequirementsBitwise)
        .WillRepeatedly(Return(calcRequirementsBitwise(requirements)));
    EXPECT_CALL(c1, initialize).Times(2);
    EXPECT_CALL(s1, getDefaultCommand).WillRepeatedly(Return(&c1));

    NiceMock<CommandMock> c2;
    EXPECT_CALL(c2, getRequirementsBitwise)
        .Times(2)  // Called when adding and removing the command
        .WillRepeatedly(Return(calcRequirementsBitwise(requirements)));
    EXPECT_CALL(c2, initialize);
    EXPECT_CALL(s1, getTestCommand).WillRepeatedly(Return(&c2));

    // Called once in the second run().
    EXPECT_CALL(c1, execute).Times(1);
    EXPECT_CALL(c1, isFinished).WillOnce(Return(false));
    EXPECT_CALL(c1, end(true)).Times(1);

    // Called once in the third run().
    EXPECT_CALL(c2, execute).Times(1);
    EXPECT_CALL(c2, isFinished).Times(3).WillRepeatedly(Return(true));
    EXPECT_CALL(c2, end(false)).Times(1);

    scheduler.registerSubsystem(&s1);
    scheduler.run();                 // Adds the default command
    scheduler.run();                 // Runs the default command
    scheduler.runHardwareTest(&s1);  // Add the test command and cancel the default command
    scheduler.run();  // Run the test command, finish it, and schedule the default command
    EXPECT_TRUE(scheduler.hasPassedTest(&s1));
    EXPECT_TRUE(scheduler.isCommandScheduled(&c1));
}

TEST(CommandScheduler, run_with_single_registered_subsystem_calls_refresh)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    SubsystemMock s1(&drivers);

    EXPECT_CALL(s1, refresh);
    EXPECT_CALL(s1, getDefaultCommand).WillOnce(Return(nullptr));

    scheduler.registerSubsystem(&s1);
    scheduler.run();
}

TEST(
    CommandScheduler,
    run_with_single_registered_subsystem_and_single_command_calls_refresh_and_execute)
{
    constexpr int RUN_TIMES = 100;

    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock s1(&drivers);
    NiceMock<CommandMock> c1;
    set<Subsystem *> subRequirementsC1{&s1};

    EXPECT_CALL(s1, refresh).Times(RUN_TIMES);
    EXPECT_CALL(c1, execute).Times(RUN_TIMES);
    EXPECT_CALL(c1, isFinished).Times(RUN_TIMES).WillRepeatedly(Return(false));
    EXPECT_CALL(c1, getRequirementsBitwise)
        .WillOnce(Return(calcRequirementsBitwise(subRequirementsC1)));
    EXPECT_CALL(c1, initialize);

    scheduler.registerSubsystem(&s1);
    scheduler.addCommand(&c1);

    for (int i = 0; i < RUN_TIMES; i++)
    {
        scheduler.run();
    }
}

TEST(CommandScheduler, run_with_many_registered_subsystems_and_commands_calls_refresh_and_execute)
{
    constexpr int RUN_TIMES = 100;
    constexpr int CMDS_AND_SUBS_TO_ADD =
        std::min(sizeof(command_scheduler_bitmap_t), sizeof(subsystem_scheduler_bitmap_t)) * 8;

    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock *subs[CMDS_AND_SUBS_TO_ADD];
    NiceMock<CommandMock> *cmds[CMDS_AND_SUBS_TO_ADD];
    set<Subsystem *> cmdRequirements[CMDS_AND_SUBS_TO_ADD];

    for (int i = 0; i < CMDS_AND_SUBS_TO_ADD; i++)
    {
        subs[i] = new SubsystemMock(&drivers);
        cmds[i] = new NiceMock<CommandMock>;
        cmdRequirements[i].emplace(subs[i]);

        EXPECT_CALL(*subs[i], refresh).Times(RUN_TIMES);
        EXPECT_CALL(*cmds[i], execute).Times(RUN_TIMES);
        EXPECT_CALL(*cmds[i], isFinished).Times(RUN_TIMES).WillRepeatedly(Return(false));
        EXPECT_CALL(*cmds[i], getRequirementsBitwise)
            .Times(CMDS_AND_SUBS_TO_ADD - i)
            .WillRepeatedly(Return(calcRequirementsBitwise(cmdRequirements[i])));
        EXPECT_CALL(*cmds[i], initialize);

        scheduler.registerSubsystem(subs[i]);
        scheduler.addCommand(cmds[i]);
    }

    uint32_t totalTime = 0;
    for (int i = 0; i < RUN_TIMES; i++)
    {
        uint32_t start = tap::arch::clock::getTimeMicroseconds();
        scheduler.run();
        uint32_t dt = tap::arch::clock::getTimeMicroseconds() - start;
        totalTime += dt;
    }
    std::cout << "             Average time to run scheduler with " << CMDS_AND_SUBS_TO_ADD
              << " subsystems and commands, " << RUN_TIMES << " samples: " << totalTime / RUN_TIMES
              << " microseconds" << std::endl;

    for (int i = 0; i < CMDS_AND_SUBS_TO_ADD; i++)
    {
        cmdRequirements[i].clear();
        delete subs[i];
        delete cmds[i];
    }
}

TEST(CommandScheduler, addCommand_disjoint_required_subsystems_successfully_added)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock s1(&drivers);
    SubsystemMock s2(&drivers);
    SubsystemMock s3(&drivers);
    SubsystemMock s4(&drivers);
    SubsystemMock s5(&drivers);
    SubsystemMock s6(&drivers);

    NiceMock<CommandMock> c1;
    NiceMock<CommandMock> c2;
    NiceMock<CommandMock> c3;

    set<Subsystem *> subRequirementsC1{&s1, &s5};
    set<Subsystem *> subRequirementsC2{&s2, &s3};
    set<Subsystem *> subRequirementsC3{&s4, &s6};

    EXPECT_CALL(c1, getRequirementsBitwise)
        .Times(3)
        .WillRepeatedly(Return(calcRequirementsBitwise(subRequirementsC1)));
    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(c2, getRequirementsBitwise)
        .Times(2)
        .WillRepeatedly(Return(calcRequirementsBitwise(subRequirementsC2)));
    EXPECT_CALL(c2, initialize);
    EXPECT_CALL(c3, getRequirementsBitwise)
        .Times(1)
        .WillRepeatedly(Return(calcRequirementsBitwise(subRequirementsC3)));
    EXPECT_CALL(c3, initialize);

    scheduler.registerSubsystem(&s1);
    scheduler.registerSubsystem(&s2);
    scheduler.registerSubsystem(&s3);
    scheduler.registerSubsystem(&s4);
    scheduler.registerSubsystem(&s5);
    scheduler.registerSubsystem(&s6);

    scheduler.addCommand(&c1);
    scheduler.addCommand(&c2);
    scheduler.addCommand(&c3);

    EXPECT_TRUE(scheduler.isCommandScheduled(&c1));
    EXPECT_TRUE(scheduler.isCommandScheduled(&c2));
    EXPECT_TRUE(scheduler.isCommandScheduled(&c3));

    EXPECT_CALL(c1, execute);
    EXPECT_CALL(c2, execute);
    EXPECT_CALL(c3, execute);
    EXPECT_CALL(c1, isFinished);
    EXPECT_CALL(c2, isFinished);
    EXPECT_CALL(c3, isFinished);
    EXPECT_CALL(s1, refresh);
    EXPECT_CALL(s2, refresh);
    EXPECT_CALL(s3, refresh);
    EXPECT_CALL(s4, refresh);
    EXPECT_CALL(s5, refresh);
    EXPECT_CALL(s6, refresh);

    scheduler.run();
}

TEST(CommandScheduler, run_large_number_of_subsystem_requirements_successful)
{
    const int NUM_DEPENDENT_SUBS = 50;

    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    NiceMock<CommandMock> c;
    SubsystemMock *subs[NUM_DEPENDENT_SUBS];
    set<Subsystem *> subRequirementsC;

    for (int i = 0; i < NUM_DEPENDENT_SUBS; i++)
    {
        subs[i] = new SubsystemMock(&drivers);
        subRequirementsC.insert(subs[i]);
        EXPECT_CALL(*subs[i], refresh);
        scheduler.registerSubsystem(subs[i]);
    }

    EXPECT_CALL(c, initialize);
    EXPECT_CALL(c, execute);
    EXPECT_CALL(c, isFinished);
    EXPECT_CALL(c, getRequirementsBitwise)
        .WillOnce(Return(calcRequirementsBitwise(subRequirementsC)));

    scheduler.addCommand(&c);
    scheduler.run();

    subRequirementsC.clear();
    for (int i = 0; i < NUM_DEPENDENT_SUBS; i++)
    {
        delete subs[i];
    }
}

TEST(
    CommandScheduler,
    run_subsystem_with_default_command_requiring_single_subsystem_added_successfully)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock s(&drivers);
    NiceMock<CommandMock> c;

    EXPECT_CALL(s, getDefaultCommand).WillOnce(Return(&c));
    EXPECT_CALL(s, refresh).Times(2);
    EXPECT_CALL(c, initialize);
    EXPECT_CALL(c, getRequirementsBitwise).WillOnce(Return(1UL << s.getGlobalIdentifier()));

    scheduler.registerSubsystem(&s);
    scheduler.run();

    EXPECT_CALL(c, execute);
    EXPECT_CALL(c, isFinished);

    scheduler.run();
}

TEST(
    CommandScheduler,
    run_subsystem_with_default_command_requiring_multiple_subsystems_added_currectly)
{
    const int NUM_DEPENDENT_SUBS = 50;

    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    NiceMock<CommandMock> c;
    SubsystemMock *subs[NUM_DEPENDENT_SUBS];
    set<Subsystem *> subRequirementsC;

    for (int i = 0; i < NUM_DEPENDENT_SUBS; i++)
    {
        subs[i] = new SubsystemMock(&drivers);
        subRequirementsC.insert(subs[i]);
        EXPECT_CALL(*subs[i], refresh).Times(2);
        scheduler.registerSubsystem(subs[i]);
    }
    EXPECT_CALL(*subs[0], getDefaultCommand).WillOnce(Return(&c));

    EXPECT_CALL(c, initialize);
    EXPECT_CALL(c, getRequirementsBitwise)
        .Times(1)
        .WillRepeatedly(Return(calcRequirementsBitwise(subRequirementsC)));

    scheduler.run();

    EXPECT_CALL(c, execute);
    EXPECT_CALL(c, isFinished);

    scheduler.run();

    subRequirementsC.clear();
    for (int i = 0; i < NUM_DEPENDENT_SUBS; i++)
    {
        delete subs[i];
    }
}

TEST(
    CommandScheduler,
    run_subsystem_with_default_command_that_depends_on_other_subsystem_that_already_has_command_is_added_successfully)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    NiceMock<CommandMock> c1;
    NiceMock<CommandMock> c2;
    SubsystemMock s1(&drivers);
    SubsystemMock s2(&drivers);
    set<Subsystem *> subRequirementsC1{&s1};
    set<Subsystem *> subRequirementsC2{&s1, &s2};

    EXPECT_CALL(c1, initialize);
    EXPECT_CALL(c1, getRequirementsBitwise)
        .Times(3)
        .WillRepeatedly(Return(calcRequirementsBitwise(subRequirementsC1)));
    EXPECT_CALL(s2, getDefaultCommand).WillOnce(Return(&c2));

    scheduler.registerSubsystem(&s1);
    scheduler.registerSubsystem(&s2);
    scheduler.addCommand(&c1);

    EXPECT_CALL(c1, execute);
    EXPECT_CALL(c1, isFinished);
    EXPECT_CALL(c1, end);
    EXPECT_CALL(c2, initialize);
    EXPECT_CALL(c2, getRequirementsBitwise)
        .WillOnce(Return(calcRequirementsBitwise(subRequirementsC2)));
    EXPECT_CALL(s1, refresh).Times(2);
    EXPECT_CALL(s2, refresh).Times(2);

    scheduler.run();

    EXPECT_CALL(c2, execute);
    EXPECT_CALL(c2, isFinished);

    scheduler.run();
}

TEST(
    CommandScheduler,
    run_command_ending_removed_from_scheduler_and_default_command_for_subsystem_automatically_added)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    NiceMock<CommandMock> c1;
    NiceMock<CommandMock> c2;
    SubsystemMock s(&drivers);
    set<Subsystem *> subRequirements{&s};

    EXPECT_CALL(c1, getRequirementsBitwise)
        .Times(2)
        .WillRepeatedly(Return(calcRequirementsBitwise(subRequirements)));
    EXPECT_CALL(c1, initialize);

    scheduler.registerSubsystem(&s);
    scheduler.addCommand(&c1);

    EXPECT_CALL(c1, execute);
    EXPECT_CALL(c1, isFinished).WillOnce(Return(true));
    EXPECT_CALL(c1, end);
    EXPECT_CALL(s, refresh).Times(2);
    EXPECT_CALL(s, getDefaultCommand).WillOnce(Return(&c2));
    EXPECT_CALL(c2, getRequirementsBitwise)
        .WillOnce(Return(calcRequirementsBitwise(subRequirements)));
    EXPECT_CALL(c2, initialize);

    scheduler.run();

    EXPECT_CALL(c2, execute);
    EXPECT_CALL(c2, isFinished).WillOnce(Return(false));

    scheduler.run();
}

TEST(CommandScheduler, run_default_command_that_naturally_ends_always_rescheduled)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    NiceMock<CommandMock> c;
    SubsystemMock s(&drivers);
    set<Subsystem *> subRequirements{&s};
    EXPECT_CALL(c, getRequirementsBitwise)
        .Times(5)
        .WillRepeatedly(Return(calcRequirementsBitwise(subRequirements)));
    EXPECT_CALL(c, initialize).Times(3);
    EXPECT_CALL(c, execute).Times(2);
    EXPECT_CALL(c, isFinished).Times(2).WillRepeatedly(Return(true));
    EXPECT_CALL(c, end).Times(2);
    EXPECT_CALL(s, refresh).Times(3);
    EXPECT_CALL(s, getDefaultCommand).Times(3).WillRepeatedly(Return(&c));

    scheduler.registerSubsystem(&s);
    scheduler.run();
    scheduler.run();
    scheduler.run();
}

TEST(CommandScheduler, run_command_ends_when_remote_disconnected)
{
    Drivers drivers;
    RemoteSafeDisconnectFunction func(&drivers);
    CommandScheduler scheduler(&drivers, true, &func);

    SubsystemMock s(&drivers);
    scheduler.registerSubsystem(&s);

    NiceMock<CommandMock> c;
    set<Subsystem *> subRequirements{&s};
    ON_CALL(c, getRequirementsBitwise)
        .WillByDefault(Return(calcRequirementsBitwise(subRequirements)));

    EXPECT_CALL(c, end);
    EXPECT_CALL(s, refreshSafeDisconnect);

    ON_CALL(drivers.remote, isConnected).WillByDefault(Return(true));
    scheduler.addCommand(&c);

    ON_CALL(drivers.remote, isConnected).WillByDefault(Return(false));
    scheduler.run();
}

TEST(CommandScheduler, run_multiple_commands_end_after_remote_disconnected)
{
    Drivers drivers;
    RemoteSafeDisconnectFunction func(&drivers);
    CommandScheduler scheduler(&drivers, true, &func);

    SubsystemMock s(&drivers);
    scheduler.registerSubsystem(&s);

    NiceMock<CommandMock> c1;
    NiceMock<CommandMock> c2;
    set<Subsystem *> subRequirements{&s};
    ON_CALL(c1, getRequirementsBitwise)
        .WillByDefault(Return(calcRequirementsBitwise(subRequirements)));
    ON_CALL(c2, getRequirementsBitwise)
        .WillByDefault(Return(calcRequirementsBitwise(subRequirements)));

    EXPECT_CALL(c1, end);
    EXPECT_CALL(c2, end);
    EXPECT_CALL(s, refreshSafeDisconnect);

    ON_CALL(drivers.remote, isConnected).WillByDefault(Return(true));
    scheduler.addCommand(&c1);
    scheduler.addCommand(&c2);

    ON_CALL(drivers.remote, isConnected).WillByDefault(Return(false));
    scheduler.run();
}

TEST(CommandScheduler, run_command_when_remote_reconnected)
{
    Drivers drivers;
    RemoteSafeDisconnectFunction func(&drivers);
    CommandScheduler scheduler(&drivers, true, &func);

    SubsystemMock s(&drivers);
    scheduler.registerSubsystem(&s);

    NiceMock<CommandMock> c;
    set<Subsystem *> subRequirements{&s};
    ON_CALL(c, getRequirementsBitwise)
        .WillByDefault(Return(calcRequirementsBitwise(subRequirements)));

    EXPECT_CALL(c, initialize).Times(2);
    EXPECT_CALL(s, refreshSafeDisconnect).Times(1);
    EXPECT_CALL(s, refresh).Times(2);
    EXPECT_CALL(c, execute).Times(2);
    EXPECT_CALL(c, end);

    ON_CALL(drivers.remote, isConnected).WillByDefault(Return(true));
    scheduler.addCommand(&c);

    ON_CALL(drivers.remote, isConnected).WillByDefault(Return(false));
    scheduler.run();

    ON_CALL(drivers.remote, isConnected).WillByDefault(Return(true));
    scheduler.addCommand(&c);
    scheduler.run();
    scheduler.run();
}

TEST(CommandScheduler, default_command_added_when_remote_reconnected)
{
    Drivers drivers;
    RemoteSafeDisconnectFunction func(&drivers);
    CommandScheduler scheduler(&drivers, true, &func);

    SubsystemMock s(&drivers);
    scheduler.registerSubsystem(&s);

    StrictMock<CommandMock> c;
    set<Subsystem *> subRequirements{&s};

    EXPECT_CALL(c, getRequirementsBitwise)
        .WillOnce(Return(calcRequirementsBitwise(subRequirements)));
    EXPECT_CALL(s, getDefaultCommand).WillOnce(Return(&c));
    EXPECT_CALL(c, isReady).WillOnce(Return(true));

    EXPECT_CALL(s, refreshSafeDisconnect).Times(1);
    EXPECT_CALL(c, initialize);
    EXPECT_CALL(s, refresh).Times(2);
    EXPECT_CALL(c, execute);
    EXPECT_CALL(c, isFinished);

    ON_CALL(drivers.remote, isConnected).WillByDefault(Return(false));
    scheduler.run();

    ON_CALL(drivers.remote, isConnected).WillByDefault(Return(true));
    scheduler.run();
    scheduler.run();
}

TEST(CommandScheduler, command_not_added_when_remote_disconnected)
{
    Drivers drivers;
    RemoteSafeDisconnectFunction func(&drivers);
    CommandScheduler scheduler(&drivers, true, &func);

    SubsystemMock s(&drivers);
    scheduler.registerSubsystem(&s);

    StrictMock<CommandMock> c;
    set<Subsystem *> subRequirements{&s};
    ON_CALL(c, getRequirementsBitwise)
        .WillByDefault(Return(calcRequirementsBitwise(subRequirements)));

    EXPECT_CALL(s, refreshSafeDisconnect);

    ON_CALL(drivers.remote, isConnected).WillByDefault(Return(false));
    scheduler.run();
}

TEST(CommandScheduler, default_command_not_added_when_remote_disconnected)
{
    Drivers drivers;
    RemoteSafeDisconnectFunction func(&drivers);
    CommandScheduler scheduler(&drivers, true, &func);

    SubsystemMock s(&drivers);
    scheduler.registerSubsystem(&s);

    StrictMock<CommandMock> c;
    set<Subsystem *> subRequirements{&s};
    ON_CALL(c, getRequirementsBitwise)
        .WillByDefault(Return(calcRequirementsBitwise(subRequirements)));

    ON_CALL(s, getDefaultCommand).WillByDefault(Return(&c));

    EXPECT_CALL(c, initialize).Times(0);
    EXPECT_CALL(s, refreshSafeDisconnect);

    ON_CALL(drivers.remote, isConnected).WillByDefault(Return(false));
    scheduler.addCommand(&c);
    scheduler.run();
}

TEST(CommandScheduler, removeCommand_nullptr_command_doesnt_crash)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    EXPECT_CALL(drivers.errorController, addToErrorList);
    scheduler.removeCommand(nullptr, false);
}

TEST(CommandScheduler, removeCommand_single_cmd_in_scheduler_removed_and_end_called)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);
    SubsystemMock sub(&drivers);
    NiceMock<CommandMock> cmd;
    EXPECT_CALL(cmd, getRequirementsBitwise)
        .Times(4)
        .WillRepeatedly(Return(calcRequirementsBitwise({&sub})));
    EXPECT_CALL(cmd, initialize).Times(2);

    scheduler.registerSubsystem(&sub);

    scheduler.addCommand(&cmd);
    EXPECT_CALL(cmd, end(true));
    scheduler.removeCommand(&cmd, true);

    scheduler.addCommand(&cmd);
    EXPECT_CALL(cmd, end(false));
    scheduler.removeCommand(&cmd, false);
}

TEST(CommandScheduler, removeCommand_single_cmd_removed_from_big_batch)
{
    static constexpr int SUBS_CMDS_TO_CREATE =
        std::min(sizeof(command_scheduler_bitmap_t), sizeof(subsystem_scheduler_bitmap_t)) * 8;
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    NiceMock<CommandMock> cmds[SUBS_CMDS_TO_CREATE];
    SubsystemMock *subs[SUBS_CMDS_TO_CREATE];

    for (uint32_t i = 0; i < SUBS_CMDS_TO_CREATE; i++)
    {
        subs[i] = new SubsystemMock(&drivers);
        EXPECT_CALL(cmds[i], getRequirementsBitwise)
            .WillRepeatedly(Return(calcRequirementsBitwise({subs[i]})));
        EXPECT_CALL(cmds[i], initialize);
        scheduler.registerSubsystem(subs[i]);
        scheduler.addCommand(&cmds[i]);
    }

    EXPECT_CALL(cmds[0], end(true));
    scheduler.removeCommand(&cmds[0], true);
    EXPECT_CALL(cmds[1], end(false));
    scheduler.removeCommand(&cmds[1], false);

    for (uint32_t i = 0; i < SUBS_CMDS_TO_CREATE; i++)
    {
        delete subs[i];
    }
}

TEST(CommandScheduler, subsystemListSize_commandListSize_returns_number_of_subs_cmds_in_scheduler)
{
    static constexpr int SUBS_CMDS_TO_CREATE =
        std::min(sizeof(command_scheduler_bitmap_t), sizeof(subsystem_scheduler_bitmap_t)) * 8;
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock *subs[SUBS_CMDS_TO_CREATE];
    NiceMock<CommandMock> cmds[SUBS_CMDS_TO_CREATE];

    for (int i = 0; i < SUBS_CMDS_TO_CREATE; i++)
    {
        EXPECT_EQ(i, scheduler.subsystemListSize());
        EXPECT_EQ(i, scheduler.commandListSize());

        subs[i] = new SubsystemMock(&drivers);
        EXPECT_CALL(cmds[i], getRequirementsBitwise)
            .WillRepeatedly(Return(calcRequirementsBitwise({subs[i]})));
        EXPECT_CALL(cmds[i], initialize);
        scheduler.registerSubsystem(subs[i]);
        scheduler.addCommand(&cmds[i]);
    }

    // Remove some commands from middle to test size calculation
    EXPECT_CALL(cmds[0], end);
    scheduler.removeCommand(&cmds[0], true);
    EXPECT_EQ(SUBS_CMDS_TO_CREATE - 1, scheduler.commandListSize());
    EXPECT_CALL(cmds[10], end);
    scheduler.removeCommand(&cmds[10], true);
    EXPECT_EQ(SUBS_CMDS_TO_CREATE - 2, scheduler.commandListSize());
    EXPECT_CALL(cmds[20], end);
    scheduler.removeCommand(&cmds[20], true);
    EXPECT_EQ(SUBS_CMDS_TO_CREATE - 3, scheduler.commandListSize());

    for (uint32_t i = 0; i < SUBS_CMDS_TO_CREATE; i++)
    {
        delete subs[i];
    }
}

TEST(CommandScheduler, iterators_nothing_in_scheduler_returns_null)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    auto cmdIt = scheduler.cmdMapBegin();
    auto subIt = scheduler.subMapBegin();

    EXPECT_EQ(nullptr, *cmdIt);
    EXPECT_EQ(nullptr, *subIt);
    EXPECT_EQ(scheduler.cmdMapEnd(), cmdIt);
    EXPECT_EQ(scheduler.subMapEnd(), subIt);
}

TEST(CommandScheduler, iterators_couple_cmd_sub_iterated_through)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    // Add a couple subs and cmds to the scheduler
    NiceMock<CommandMock> cmd1;
    NiceMock<CommandMock> cmd2;
    SubsystemMock sub1(&drivers);
    SubsystemMock sub2(&drivers);

    EXPECT_CALL(cmd1, initialize);
    EXPECT_CALL(cmd2, initialize);
    EXPECT_CALL(cmd1, getRequirementsBitwise)
        .Times(2)
        .WillRepeatedly(Return(calcRequirementsBitwise({&sub1})));
    EXPECT_CALL(cmd2, getRequirementsBitwise).WillOnce(Return(calcRequirementsBitwise({&sub2})));

    scheduler.registerSubsystem(&sub1);
    scheduler.registerSubsystem(&sub2);
    scheduler.addCommand(&cmd1);
    scheduler.addCommand(&cmd2);

    auto cmdIt = scheduler.cmdMapBegin();
    EXPECT_EQ(&cmd1, *cmdIt);
    EXPECT_EQ(&cmd2, *(++cmdIt));
    cmdIt++;
    EXPECT_EQ(nullptr, *cmdIt);

    auto subIt = scheduler.subMapBegin();
    EXPECT_EQ(&sub1, *subIt);
    EXPECT_EQ(&sub2, *(++subIt));
    subIt++;
    EXPECT_EQ(nullptr, *subIt);
}

TEST(CommandScheduler, iterators_many_cmds_subs_iterated_through_using_foreach)
{
    static constexpr int SUBS_CMDS_TO_CREATE =
        std::min(sizeof(command_scheduler_bitmap_t), sizeof(subsystem_scheduler_bitmap_t)) * 8;
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock *subs[SUBS_CMDS_TO_CREATE];
    NiceMock<CommandMock> cmds[SUBS_CMDS_TO_CREATE];

    for (uint32_t i = 0; i < SUBS_CMDS_TO_CREATE; i++)
    {
        subs[i] = new SubsystemMock(&drivers);
        EXPECT_CALL(cmds[i], getRequirementsBitwise)
            .WillRepeatedly(Return(calcRequirementsBitwise({subs[i]})));
        EXPECT_CALL(cmds[i], initialize);
        scheduler.registerSubsystem(subs[i]);
        scheduler.addCommand(&cmds[i]);
    }

    int i = 0;
    // Foreach with subsystem map
    std::for_each(scheduler.subMapBegin(), scheduler.subMapEnd(), [&](Subsystem *sub) {
        EXPECT_EQ(subs[i], sub);
        i++;
    });

    i = 0;
    // Foreach with command map
    std::for_each(scheduler.cmdMapBegin(), scheduler.cmdMapEnd(), [&](Command *cmd) {
        EXPECT_EQ(&cmds[i], cmd);
        i++;
    });

    for (uint32_t i = 0; i < SUBS_CMDS_TO_CREATE; i++)
    {
        delete subs[i];
    }
}

TEST(CommandScheduler, iterators_work_properly_with_gaps_in_global_registrar)
{
    static constexpr int SUBS_CMDS_TO_CREATE =
        std::min(sizeof(subsystem_scheduler_bitmap_t), sizeof(command_scheduler_bitmap_t)) * 8;

    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock *subs[SUBS_CMDS_TO_CREATE];
    NiceMock<CommandMock> cmds[SUBS_CMDS_TO_CREATE];

    for (uint32_t i = 0; i < SUBS_CMDS_TO_CREATE; i++)
    {
        subs[i] = new SubsystemMock(&drivers);
        if (i % 2 == 0)
        {
            scheduler.registerSubsystem(subs[i]);
        }
        else
        {
            EXPECT_CALL(cmds[i], getRequirementsBitwise)
                .WillRepeatedly(Return(calcRequirementsBitwise({subs[i - 1]})));
            EXPECT_CALL(cmds[i], initialize);
            scheduler.addCommand(&cmds[i]);
        }
    }

    int i = 0;
    for (auto it = scheduler.subMapBegin(); it != scheduler.subMapEnd(); it++)
    {
        EXPECT_EQ(subs[2 * i], *it);
        i++;
    }

    i = 0;
    for (auto it = scheduler.cmdMapBegin(); it != scheduler.cmdMapEnd(); it++)
    {
        EXPECT_EQ(&cmds[2 * i + 1], *it);
        i++;
    }

    for (uint32_t i = 0; i < SUBS_CMDS_TO_CREATE; i++)
    {
        delete subs[i];
    }
}

TEST(CommandScheduler, subsystem_iterator_iterator_begin_with_invalid_index)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    SubsystemMock sub1(&drivers);
    SubsystemMock sub2(&drivers);

    // Only add the second subsystem, so that the subsystem iterator starts on a subsystem that
    // is not in the scheduler
    scheduler.registerSubsystem(&sub2);

    auto it = scheduler.subMapBegin();
    EXPECT_EQ(&sub2, *it);
    EXPECT_EQ(scheduler.subMapEnd(), ++it);
}

TEST(CommandScheduler, addCommand_doesnt_add_if_command_not_ready)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers, true);

    NiceMock<CommandMock> c1;

    EXPECT_CALL(c1, isReady).WillOnce(Return(false));

    scheduler.addCommand(&c1);
    bool isScheduled = scheduler.isCommandScheduled(&c1);

    EXPECT_EQ(isScheduled, false);
}

class TestComprisedCommand : public ComprisedCommand
{
    NiceMock<CommandMock> *c1;
    NiceMock<CommandMock> *c2;

public:
    TestComprisedCommand(
        Drivers *drivers,
        NiceMock<CommandMock> *c1,
        NiceMock<CommandMock> *c2,
        SubsystemMock *s1,
        SubsystemMock *s2,
        SubsystemMock *s3)
        : ComprisedCommand(drivers),
          c1(c1),
          c2(c2)
    {
        this->comprisedCommandScheduler.registerSubsystem(s1);
        this->comprisedCommandScheduler.registerSubsystem(s2);
        this->comprisedCommandScheduler.registerSubsystem(s3);
    }

    void initialize() override
    {
        this->comprisedCommandScheduler.addCommand(c1);
        this->comprisedCommandScheduler.addCommand(c2);
    }

    void execute() override
    {
        // this->comprisedCommandScheduler.addCommand(c1);
        // this->comprisedCommandScheduler.addCommand(c2);
        this->comprisedCommandScheduler.run();
    }

    void end(bool interrupted) override
    {
        this->comprisedCommandScheduler.removeCommand(c1, interrupted);
        this->comprisedCommandScheduler.removeCommand(c2, interrupted);
    }

    bool isFinished() const override
    {
        return this->comprisedCommandScheduler.isCommandScheduled(c1) &&
               this->comprisedCommandScheduler.isCommandScheduled(c2);
    }

    const char *getName() const override { return "test comprised command"; }
};

TEST(CommandScheduler, refreshSafeDisconnect_only_called_once)
{
    Drivers drivers;
    RemoteSafeDisconnectFunction func(&drivers);
    CommandScheduler scheduler(&drivers, true, &func);

    SubsystemMock s1(&drivers);
    SubsystemMock s2(&drivers);
    SubsystemMock s3(&drivers);
    scheduler.registerSubsystem(&s1);
    scheduler.registerSubsystem(&s2);
    scheduler.registerSubsystem(&s3);

    NiceMock<CommandMock> c1;
    NiceMock<CommandMock> c2;
    set<Subsystem *> subRequirementsC1{&s1, &s2};
    set<Subsystem *> subRequirementsC2{&s3};
    ON_CALL(c1, getRequirementsBitwise)
        .WillByDefault(Return(calcRequirementsBitwise(subRequirementsC1)));
    ON_CALL(c2, getRequirementsBitwise)
        .WillByDefault(Return(calcRequirementsBitwise(subRequirementsC2)));

    TestComprisedCommand cc1(&drivers, &c1, &c2, &s1, &s2, &s3);

    ON_CALL(drivers.remote, isConnected).WillByDefault(Return(false));
    scheduler.addCommand(&cc1);

    EXPECT_CALL(s1, refreshSafeDisconnect).Times(1);
    EXPECT_CALL(s2, refreshSafeDisconnect).Times(1);
    EXPECT_CALL(s3, refreshSafeDisconnect).Times(1);

    scheduler.run();
}
