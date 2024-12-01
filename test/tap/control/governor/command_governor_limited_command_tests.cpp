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

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/command_governor_interface_mock.hpp"
#include "tap/mock/command_mock.hpp"
#include "tap/mock/subsystem_mock.hpp"

using namespace tap::control::governor;
using namespace testing;
using namespace tap::mock;
using namespace tap;

template <typename T>
class GovernorLimitedCommandTest : public Test
{
protected:
    static constexpr size_t S = T::value;

    GovernorLimitedCommandTest() : sub(&drivers) {}

    void SetUp() override
    {
        ON_CALL(cmdToGovern, getName).WillByDefault(Return("great name"));
        ON_CALL(cmdToGovern, getRequirementsBitwise)
            .WillByDefault(Return(1UL << sub.getGlobalIdentifier()));

        std::array<CommandGovernorInterface *, S> governorPtrs;

        for (size_t i = 0; i < governors.size(); i++)
        {
            governorPtrs[i] = &governors[i];
        }

        cmd = std::shared_ptr<GovernorLimitedCommand<S>>(
            new GovernorLimitedCommand<S>({&sub}, cmdToGovern, governorPtrs));
    }

    Drivers drivers;
    NiceMock<CommandMock> cmdToGovern;
    NiceMock<SubsystemMock> sub;

    std::array<NiceMock<CommandGovernorInterfaceMock>, S> governors;

    std::shared_ptr<GovernorLimitedCommand<S>> cmd;
};

using TestTypes = Types<
    std::integral_constant<std::size_t, 1>,
    std::integral_constant<std::size_t, 2>,
    std::integral_constant<std::size_t, 5>>;

TYPED_TEST_SUITE(GovernorLimitedCommandTest, TestTypes);

TYPED_TEST(GovernorLimitedCommandTest, getName_ret_cmdToGovern_name)
{
    EXPECT_STREQ(TestFixture::cmdToGovern.getName(), TestFixture::cmd->getName());
}

TYPED_TEST(GovernorLimitedCommandTest, isReady_false_cmd_not_ready)
{
    for (auto &gov : TestFixture::governors)
    {
        ON_CALL(gov, isReady).WillByDefault(Return(true));
    }

    ON_CALL(TestFixture::cmdToGovern, isReady).WillByDefault(Return(false));

    EXPECT_FALSE(TestFixture::cmd->isReady());
}

TYPED_TEST(GovernorLimitedCommandTest, isReady_false_when_govs_not_ready)
{
    for (auto &gov : TestFixture::governors)
    {
        ON_CALL(gov, isReady).WillByDefault(Return(false));
    }

    ON_CALL(TestFixture::cmdToGovern, isReady).WillByDefault(Return(true));

    EXPECT_FALSE(TestFixture::cmd->isReady());
}

TYPED_TEST(GovernorLimitedCommandTest, isReady_false_when_single_gov_not_ready)
{
    ON_CALL(TestFixture::governors[0], isReady).WillByDefault(Return(false));

    for (size_t i = 1; i < TestFixture::governors.size(); i++)
    {
        ON_CALL(TestFixture::governors[i], isReady).WillByDefault(Return(true));
    }

    ON_CALL(TestFixture::cmdToGovern, isReady).WillByDefault(Return(true));

    EXPECT_FALSE(TestFixture::cmd->isReady());
}

TYPED_TEST(GovernorLimitedCommandTest, isReady_true_when_gov_and_cmd_ready)
{
    for (auto &gov : TestFixture::governors)
    {
        ON_CALL(gov, isReady).WillByDefault(Return(true));
    }

    ON_CALL(TestFixture::cmdToGovern, isReady).WillByDefault(Return(true));

    EXPECT_TRUE(TestFixture::cmd->isReady());
}

TYPED_TEST(GovernorLimitedCommandTest, isFinished_true_when_cmd_finished)
{
    for (auto &gov : TestFixture::governors)
    {
        ON_CALL(gov, isFinished).WillByDefault(Return(false));
    }

    ON_CALL(TestFixture::cmdToGovern, isFinished).WillByDefault(Return(true));

    EXPECT_TRUE(TestFixture::cmd->isFinished());
}

TYPED_TEST(GovernorLimitedCommandTest, isFinished_true_when_govs_finished)
{
    for (auto &gov : TestFixture::governors)
    {
        ON_CALL(gov, isFinished).WillByDefault(Return(true));
    }

    ON_CALL(TestFixture::cmdToGovern, isFinished).WillByDefault(Return(false));

    EXPECT_TRUE(TestFixture::cmd->isFinished());
}

TYPED_TEST(GovernorLimitedCommandTest, isFinished_false_when_govs_not_finished)
{
    for (auto &gov : TestFixture::governors)
    {
        ON_CALL(gov, isFinished).WillByDefault(Return(false));
    }

    ON_CALL(TestFixture::cmdToGovern, isFinished).WillByDefault(Return(false));

    EXPECT_FALSE(TestFixture::cmd->isFinished());
}

TYPED_TEST(GovernorLimitedCommandTest, initialize_execute_end_runs_cmdToGovern)
{
    InSequence seq;
    EXPECT_CALL(TestFixture::cmdToGovern, initialize);
    EXPECT_CALL(TestFixture::cmdToGovern, execute);
    EXPECT_CALL(TestFixture::cmdToGovern, end(false));
    EXPECT_CALL(TestFixture::cmdToGovern, end(true));

    for (auto &gov : TestFixture::governors)
    {
        ON_CALL(gov, isFinished).WillByDefault(Return(false));
    }

    TestFixture::cmd->initialize();
    TestFixture::cmd->execute();
    TestFixture::cmd->end(false);
    TestFixture::cmd->end(true);
}
