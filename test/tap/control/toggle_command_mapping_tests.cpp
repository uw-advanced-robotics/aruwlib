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

#include "tap/control/remote_map_state.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/drivers.hpp"

#include "test_command.hpp"
#include "test_subsystem.hpp"

using namespace tap::control;
using namespace testing;
using tap::Drivers;
using namespace tap::communication::serial;

// Adding command with switch state RemoteMapState (RMS)

TEST(
    ToggleCommandMapping,
    executeCommandMapping_single_command_not_added_if_switch_based_RMS_is_not_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2;
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand).Times(0);

    commandMapping.executeCommandMapping(ms2);
}

TEST(ToggleCommandMapping, executeCommandMapping_single_command_added_if_switch_based_RMS_is_equal)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2 = ms1;
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);

    commandMapping.executeCommandMapping(ms2);
}

TEST(ToggleCommandMapping, executeCommandMapping_single_command_added_if_switch_based_RMS_is_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2 = ms1;
    ms2.initKeys(42);
    ms2.initRSwitch(Remote::SwitchState::UP);
    ms2.initLMouseButton();
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);

    commandMapping.executeCommandMapping(ms2);
}

// Adding commands, key based (including neg keys) RMS

TEST(
    ToggleCommandMapping,
    executeCommandMapping_single_command_not_added_if_key_based_RMS_is_not_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B});
    RemoteMapState ms2;
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand).Times(0);

    commandMapping.executeCommandMapping(ms2);
}

TEST(ToggleCommandMapping, executeCommandMapping_single_command_added_if_key_based_RMS_is_equal)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B});
    RemoteMapState ms2 = ms1;
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);

    commandMapping.executeCommandMapping(ms2);
}

TEST(ToggleCommandMapping, executeCommandMapping_single_command_added_if_key_based_RMS_is_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B});
    RemoteMapState ms2 = ms1;
    ms2.initLMouseButton();
    ms2.initLSwitch(Remote::SwitchState::DOWN);
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);

    commandMapping.executeCommandMapping(ms2);
}

TEST(
    ToggleCommandMapping,
    executeCommandMapping_single_command_not_added_if_key_based_RMS_with_neg_keys_contains_matching_neg_keys)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B}, {Remote::Key::C, Remote::Key::D});
    RemoteMapState ms2({Remote::Key::A, Remote::Key::B, Remote::Key::C, Remote::Key::D});
    ms2.initLMouseButton();
    ms2.initLSwitch(Remote::SwitchState::DOWN);
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand).Times(0);

    commandMapping.executeCommandMapping(ms2);
}

// Second stage of toggle command, match RMS to add command, then don't match RMS and match again,
// which should remove the command

TEST(ToggleCommandMapping, executeCommandMapping_single_command_removed_if_switch_based_RMS_toggled)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2 = ms1;
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    bool cmdScheduled = false;
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc))
        .Times(1)
        .WillRepeatedly([&](Command *) { cmdScheduled = true; });
    EXPECT_CALL(drivers.commandScheduler, removeCommand(&tc, false))
        .Times(1)
        .WillRepeatedly([&](Command *, bool) { cmdScheduled = false; });
    ON_CALL(drivers.commandScheduler, isCommandScheduled)
        .WillByDefault(ReturnPointee(&cmdScheduled));

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState();
    commandMapping.executeCommandMapping(ms2);
    ms2 = ms1;
    commandMapping.executeCommandMapping(ms2);
}

TEST(
    ToggleCommandMapping,
    executeCommandMapping_single_command_not_removed_if_switch_based_RMS_never_equal_again)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2 = ms1;
    ms2.initKeys(42);
    ms2.initLMouseButton();
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    bool cmdScheduled = false;
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc))
        .Times(1)
        .WillRepeatedly([&](Command *) { cmdScheduled = true; });
    EXPECT_CALL(drivers.commandScheduler, removeCommand).Times(0);
    ON_CALL(drivers.commandScheduler, isCommandScheduled)
        .WillByDefault(ReturnPointee(&cmdScheduled));

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    commandMapping.executeCommandMapping(ms2);
    commandMapping.executeCommandMapping(ms2);
}

TEST(ToggleCommandMapping, executeCommandMapping_single_command_removed_if_key_based_RMS_toggled)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B}, {});
    RemoteMapState ms2({Remote::Key::A, Remote::Key::B, Remote::Key::C}, {});
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    bool cmdScheduled = false;
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc))
        .Times(1)
        .WillRepeatedly([&](Command *) { cmdScheduled = true; });
    EXPECT_CALL(drivers.commandScheduler, removeCommand)
        .Times(1)
        .WillRepeatedly([&](Command *, bool) { cmdScheduled = false; });
    ON_CALL(drivers.commandScheduler, isCommandScheduled)
        .WillByDefault(ReturnPointee(&cmdScheduled));

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState();
    commandMapping.executeCommandMapping(ms2);
    ms2 = ms1;
    commandMapping.executeCommandMapping(ms2);
}

TEST(
    ToggleCommandMapping,
    executeCommandMapping_single_command_removed_if_keys_match_then_neg_keys_match)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B}, {Remote::Key::C, Remote::Key::D});
    RemoteMapState ms2({Remote::Key::A, Remote::Key::B}, {});
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    bool cmdScheduled = false;
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc))
        .Times(2)
        .WillRepeatedly([&](Command *) { cmdScheduled = true; });
    EXPECT_CALL(drivers.commandScheduler, removeCommand(&tc, false))
        .Times(2)
        .WillRepeatedly([&](Command *, bool) { cmdScheduled = false; });
    ON_CALL(drivers.commandScheduler, isCommandScheduled)
        .WillByDefault(ReturnPointee(&cmdScheduled));

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState(
        {Remote::Key::A, Remote::Key::B, Remote::Key::C, Remote::Key::D, Remote::Key::E});
    commandMapping.executeCommandMapping(ms2);  // command is now removed
    ms2 = RemoteMapState({Remote::Key::A, Remote::Key::B});
    commandMapping.executeCommandMapping(ms2);  // command is now added
    ms2 = RemoteMapState();
    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState(
        {Remote::Key::A, Remote::Key::B, Remote::Key::C, Remote::Key::D, Remote::Key::E});
    commandMapping.executeCommandMapping(ms2);  // command is now removed
}

TEST(
    ToggleCommandMapping,
    executeCommandMapping_after_neg_key_removes_command_toggle_works_as_expected)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B}, {Remote::Key::C, Remote::Key::D});
    RemoteMapState ms2({Remote::Key::A, Remote::Key::B}, {});
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    bool cmdScheduled = false;
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc))
        .Times(2)
        .WillRepeatedly([&](Command *) { cmdScheduled = true; });
    EXPECT_CALL(drivers.commandScheduler, removeCommand(&tc, false))
        .Times(2)
        .WillRepeatedly([&](Command *, bool) { cmdScheduled = false; });
    ON_CALL(drivers.commandScheduler, isCommandScheduled)
        .WillByDefault(ReturnPointee(&cmdScheduled));

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState(
        {Remote::Key::A, Remote::Key::B, Remote::Key::C, Remote::Key::D, Remote::Key::E});
    commandMapping.executeCommandMapping(ms2);  // command is now removed
    ms2 = RemoteMapState({Remote::Key::A, Remote::Key::B, Remote::Key::E});
    commandMapping.executeCommandMapping(ms2);  // command is now added
    ms2 = RemoteMapState();
    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState({Remote::Key::A, Remote::Key::B, Remote::Key::C});
    commandMapping.executeCommandMapping(ms2);  // command is now removed
}

TEST(
    ToggleCommandMapping,
    executeCommandMapping_toggle_and_press_state_exited_after_cmd_naturally_ends)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A});
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    bool cmdScheduled = false;
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(2);
    EXPECT_CALL(drivers.commandScheduler, removeCommand(_, _)).Times(0);
    ON_CALL(drivers.commandScheduler, isCommandScheduled)
        .WillByDefault(ReturnPointee(&cmdScheduled));

    // command should be added since mapping matches
    commandMapping.executeCommandMapping(ms1);
    cmdScheduled = false;
    // command now not scheduled, when map state matches again, it should be added again and not
    // removed
    commandMapping.executeCommandMapping(ms1);
}

TEST(
    ToggleCommandMapping,
    executeCommandMapping_toggle_and_press_state_exited_after_one_of_two_cmds_naturally_ends)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc1(&ts);
    TestCommand tc2(&ts);
    RemoteMapState ms1({Remote::Key::A});
    bool cmdScheduled = false;
    ToggleCommandMapping commandMapping(&drivers, {&tc1, &tc2}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc1)).Times(2);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc2)).Times(2);
    EXPECT_CALL(drivers.commandScheduler, removeCommand(_, _)).Times(0);
    ON_CALL(drivers.commandScheduler, isCommandScheduled(&tc1))
        .WillByDefault(ReturnPointee(&cmdScheduled));
    ON_CALL(drivers.commandScheduler, isCommandScheduled(&tc2))
        .WillByDefault(ReturnPointee(&cmdScheduled));

    // command should be added since mapping matches
    commandMapping.executeCommandMapping(ms1);
    // all commands not scheduled, should be restarted
    cmdScheduled = false;
    commandMapping.executeCommandMapping(ms1);
}

TEST(
    ToggleCommandMapping,
    executeCommandMapping_toggle_and_not_press_state_exited_after_cmd_naturally_ends)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A});
    RemoteMapState ms2 = RemoteMapState();
    ToggleCommandMapping commandMapping(&drivers, {&tc}, ms1);
    bool cmdScheduled = false;
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc))
        .Times(2)
        .WillRepeatedly([&](Command *) { cmdScheduled = true; });
    EXPECT_CALL(drivers.commandScheduler, removeCommand(&tc, false))
        .Times(1)
        .WillRepeatedly([&](Command *, bool) { cmdScheduled = false; });
    ON_CALL(drivers.commandScheduler, isCommandScheduled)
        .WillByDefault(ReturnPointee(&cmdScheduled));

    // command should be added since mapping matches
    commandMapping.executeCommandMapping(ms1);
    // in not pressed state and still in toggled state
    commandMapping.executeCommandMapping(ms2);
    cmdScheduled = false;
    // command now not scheduled, when map state matches again, it should be added again and not
    // removed
    commandMapping.executeCommandMapping(ms1);

    // ensure in correct state by cycling through toggle commands states
    commandMapping.executeCommandMapping(ms2);
    commandMapping.executeCommandMapping(ms1);
}
