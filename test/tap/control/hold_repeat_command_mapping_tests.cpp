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

#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/remote_map_state.hpp"
#include "tap/drivers.hpp"

#include "test_command.hpp"
#include "test_subsystem.hpp"

using namespace tap::control;
using namespace testing;
using tap::Drivers;
using namespace tap::communication::serial;

// A HoldRepetCommandMapping should behave in every way like a HoldCommandMapping except in cases
// where the command is completed while the hold mapping is still valid.

// Adding command with switch state RemoteMapState (RMS)

TEST(
    HoldRepeatCommandMapping,
    executeCommandMapping_single_command_not_added_if_switch_based_RMS_is_not_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2;
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc}, ms1, true);
    EXPECT_CALL(drivers.commandScheduler, addCommand).Times(0);

    commandMapping.executeCommandMapping(ms2);
}

TEST(
    HoldRepeatCommandMapping,
    executeCommandMapping_single_command_added_if_switch_based_RMS_is_equal)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2 = ms1;
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc}, ms1, true);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, isCommandScheduled).Times(1);

    commandMapping.executeCommandMapping(ms2);
}

TEST(
    HoldRepeatCommandMapping,
    executeCommandMapping_single_command_added_if_switch_based_RMS_is_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2 = ms1;
    ms2.initKeys(42);
    ms2.initRSwitch(Remote::SwitchState::UP);
    ms2.initLMouseButton();
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc}, ms1, true);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, isCommandScheduled).Times(1);

    commandMapping.executeCommandMapping(ms2);
}

// Adding commands, key based (including neg keys) RMS

TEST(
    HoldRepeatCommandMapping,
    executeCommandMapping_single_command_not_added_if_key_based_RMS_is_not_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B});
    RemoteMapState ms2;
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc}, ms1, true);
    EXPECT_CALL(drivers.commandScheduler, addCommand).Times(0);

    commandMapping.executeCommandMapping(ms2);
}

TEST(HoldRepeatCommandMapping, executeCommandMapping_single_command_added_if_key_based_RMS_is_equal)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B});
    RemoteMapState ms2 = ms1;
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc}, ms1, true);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, isCommandScheduled).Times(1);

    commandMapping.executeCommandMapping(ms2);
}

TEST(
    HoldRepeatCommandMapping,
    executeCommandMapping_single_command_added_if_key_based_RMS_is_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B});
    RemoteMapState ms2 = ms1;
    ms2.initLMouseButton();
    ms2.initLSwitch(Remote::SwitchState::DOWN);
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc}, ms1, true);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, isCommandScheduled).Times(1);

    commandMapping.executeCommandMapping(ms2);
}

TEST(
    HoldRepeatCommandMapping,
    executeCommandMapping_single_command_not_added_if_key_based_RMS_with_neg_keys_contains_matching_neg_keys)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B}, {Remote::Key::C, Remote::Key::D});
    RemoteMapState ms2({Remote::Key::A, Remote::Key::B, Remote::Key::C, Remote::Key::D});
    ms2.initLMouseButton();
    ms2.initLSwitch(Remote::SwitchState::DOWN);
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc}, ms1, true);
    EXPECT_CALL(drivers.commandScheduler, addCommand).Times(0);

    commandMapping.executeCommandMapping(ms2);
}

// Removing already-added command from scheduler, switches and keys (including neg keys)

TEST(
    HoldRepeatCommandMapping,
    executeCommandMapping_single_command_removed_if_switch_based_RMS_no_longer_equal)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2 = ms1;
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc}, ms1, true);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, removeCommand(&tc, false)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, isCommandScheduled(&tc)).Times(1);

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState();
    commandMapping.executeCommandMapping(ms2);
}

TEST(
    HoldRepeatCommandMapping,
    executeCommandMapping_single_command_not_removed_if_switch_based_RMS_no_longer_equal_but_endCommandsWhenNotHeld_false)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2 = ms1;
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc}, ms1, false);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, removeCommand(&tc, testing::_)).Times(0);
    EXPECT_CALL(drivers.commandScheduler, isCommandScheduled(&tc)).Times(1);

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState();
    commandMapping.executeCommandMapping(ms2);
}

TEST(
    HoldRepeatCommandMapping,
    executeCommandMapping_single_command_not_removed_if_switch_based_RMS_still_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2 = ms1;
    ms2.initKeys(42);
    ms2.initLMouseButton();
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc}, ms1, true);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, removeCommand).Times(0);
    EXPECT_CALL(drivers.commandScheduler, isCommandScheduled).Times(2);

    commandMapping.executeCommandMapping(ms2);
    ON_CALL(drivers.commandScheduler, isCommandScheduled)
        .WillByDefault([](const Command *) { return true; });
    ms2.initRMouseButton();
    commandMapping.executeCommandMapping(ms2);
}

TEST(
    HoldRepeatCommandMapping,
    executeCommandMapping_single_command_not_removed_if_key_based_RMS_still_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B}, {});
    RemoteMapState ms2({Remote::Key::A, Remote::Key::B, Remote::Key::C}, {});
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc}, ms1, true);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, removeCommand).Times(0);
    EXPECT_CALL(drivers.commandScheduler, isCommandScheduled).Times(2);

    commandMapping.executeCommandMapping(ms2);
    ON_CALL(drivers.commandScheduler, isCommandScheduled)
        .WillByDefault([](const Command *) { return true; });
    ms2 = ms1;
    commandMapping.executeCommandMapping(ms2);
}

TEST(
    HoldRepeatCommandMapping,
    executeCommandMapping_single_command_removed_if_key_based_RMS_not_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B}, {});
    RemoteMapState ms2({Remote::Key::A, Remote::Key::B, Remote::Key::C}, {});
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc}, ms1, true);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, removeCommand(&tc, false)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, isCommandScheduled(&tc)).Times(1);

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState({Remote::Key::A, Remote::Key::F});
    commandMapping.executeCommandMapping(ms2);
}

TEST(
    HoldRepeatCommandMapping,
    executeCommandMapping_single_command_not_removed_if_key_based_RMS_does_not_matche_neg_keys)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B}, {Remote::Key::C});
    RemoteMapState ms2({Remote::Key::A, Remote::Key::B, Remote::Key::E});
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc}, ms1, true);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, removeCommand).Times(0);
    EXPECT_CALL(drivers.commandScheduler, isCommandScheduled(&tc)).Times(2);

    commandMapping.executeCommandMapping(ms2);
    ON_CALL(drivers.commandScheduler, isCommandScheduled(&tc))
        .WillByDefault([](const Command *) { return true; });
    ms2 = RemoteMapState({Remote::Key::A, Remote::Key::B, Remote::Key::F});
    commandMapping.executeCommandMapping(ms2);
}

TEST(
    HoldRepeatCommandMapping,
    executeCommandMapping_single_command_removed_if_key_based_RMS_matches_neg_keys)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B}, {Remote::Key::C});
    RemoteMapState ms2({Remote::Key::A, Remote::Key::B, Remote::Key::E});
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc}, ms1, true);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, removeCommand(&tc, false)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, isCommandScheduled(&tc)).Times(1);

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState({Remote::Key::A, Remote::Key::B, Remote::Key::C});
    commandMapping.executeCommandMapping(ms2);
}

TEST(
    HoldRepeatCommandMapping,
    executeCommandMapping_multiple_commands_added_and_removed_if_RMS_matches_then_does_not_match)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc1(&ts);
    TestCommand tc2(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc1, &tc2}, ms1, true);
    EXPECT_CALL(drivers.commandScheduler, addCommand).Times(2);
    EXPECT_CALL(drivers.commandScheduler, removeCommand).Times(2);
    EXPECT_CALL(drivers.commandScheduler, isCommandScheduled).Times(2);

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);
    commandMapping.executeCommandMapping(ms2);
}

// Tests involving the completion of commands while the mapping is still valid

TEST(HoldRepeatCommandMapping, executeCommandMapping_single_command_readded_if_command_finishes)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc}, ms1, true);
    EXPECT_CALL(drivers.commandScheduler, isCommandScheduled(&tc)).Times(2);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(2);

    commandMapping.executeCommandMapping(ms2);
    commandMapping.executeCommandMapping(ms2);
}

TEST(
    HoldRepeatCommandMapping,
    executeCommandMapping_multiple_commands_readded_if_they_finish_independently_of_each_other)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc1(&ts);
    TestCommand tc2(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2 = ms1;
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc1, &tc2}, ms1, true);
    EXPECT_CALL(drivers.commandScheduler, isCommandScheduled(&tc1)).Times(3);
    EXPECT_CALL(drivers.commandScheduler, isCommandScheduled(&tc2)).Times(3);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc1)).Times(2);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc2)).Times(2);

    commandMapping.executeCommandMapping(ms2);
    // tc1 command added, tc2 not added
    ON_CALL(drivers.commandScheduler, isCommandScheduled(&tc1))
        .WillByDefault([](const Command *) { return true; });
    commandMapping.executeCommandMapping(ms2);
    // tc1 not added, tc2 added
    ON_CALL(drivers.commandScheduler, isCommandScheduled(&tc1))
        .WillByDefault([](const Command *) { return false; });
    ON_CALL(drivers.commandScheduler, isCommandScheduled(&tc2))
        .WillByDefault([](const Command *) { return true; });
    commandMapping.executeCommandMapping(ms2);
}

TEST(HoldRepeatCommandMapping, executeCommandMapping_maxTimesToSchedule_zero_no_commands_scheduled)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc}, ms, false, 0);

    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(0);
    ON_CALL(drivers.commandScheduler, isCommandScheduled).WillByDefault(Return(false));

    commandMapping.executeCommandMapping(ms);
}

TEST(HoldRepeatCommandMapping, executeCommandMapping_maxTimesToSchedule_1_command_scheduled_once)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc}, ms, false, 1);

    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    ON_CALL(drivers.commandScheduler, isCommandScheduled).WillByDefault(Return(false));

    commandMapping.executeCommandMapping(ms);
    commandMapping.executeCommandMapping(ms);
}

TEST(
    HoldRepeatCommandMapping,
    executeCommandMapping_maxTimesToSchedule_1_both_commands_scheduled_once)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc1(&ts);
    TestCommand tc2(&ts);
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc1, &tc2}, ms, false, 1);

    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc1)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc2)).Times(1);
    ON_CALL(drivers.commandScheduler, isCommandScheduled).WillByDefault(Return(false));

    commandMapping.executeCommandMapping(ms);
    commandMapping.executeCommandMapping(ms);
}

TEST(HoldRepeatCommandMapping, executeCommandMapping_maxTimesToSchedule_2_schedules_command_twice)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc}, ms, false, 2);

    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(2);
    ON_CALL(drivers.commandScheduler, isCommandScheduled).WillByDefault(Return(false));

    commandMapping.executeCommandMapping(ms);
    commandMapping.executeCommandMapping(ms);
    commandMapping.executeCommandMapping(ms);
}

TEST(
    HoldRepeatCommandMapping,
    executeCommandMapping_maxTimesToSchedule_changes_from_1_to_2_allows_command_to_be_rescheduled_if_held)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    HoldRepeatCommandMapping commandMapping(&drivers, {&tc}, ms, false, 1);

    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(2);
    ON_CALL(drivers.commandScheduler, isCommandScheduled).WillByDefault(Return(false));

    commandMapping.executeCommandMapping(ms);
    commandMapping.setMaxTimesToSchedule(2);
    commandMapping.executeCommandMapping(ms);
    commandMapping.executeCommandMapping(ms);
}
