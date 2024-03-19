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

#include <gtest/gtest.h>

#include "tap/architecture/clock.hpp"
#include "tap/motor/motorsim/dji_motor_sim_handler.hpp"

#include "modm/architecture/interface/can_message.hpp"

using namespace testing;
using namespace tap::motor::motorsim;
using namespace tap::motor;
using namespace tap::can;

class DjiMotorSimHandlerTest : public Test
{
protected:
    DjiMotorSimHandler handler;
    tap::arch::clock::ClockStub clock;
};

TEST_F(DjiMotorSimHandlerTest, registering_sims_then_resetting)
{
    std::shared_ptr<MotorSim> sim1(new MotorSim(MotorSim::M3508_CONFIG));
    std::shared_ptr<MotorSim> sim2(new MotorSim(MotorSim::M3508_CONFIG));

    sim1->setMotorInput(MotorSim::M3508_CONFIG.maxInputMag);
    sim2->setMotorInput(-MotorSim::M3508_CONFIG.maxInputMag);

    handler.registerSim(sim1, std::tuple<CanBus, MotorId>(CanBus::CAN_BUS1, MOTOR2));
    handler.registerSim(sim2, std::tuple<CanBus, MotorId>(CanBus::CAN_BUS2, MOTOR5));

    handler.resetMotorSims();

    EXPECT_EQ(0, sim1->getCurrent());
    EXPECT_EQ(0, sim2->getCurrent());
}

TEST_F(DjiMotorSimHandlerTest, parseMotorMessage_no_sims_registered)
{
    modm::can::Message msg(static_cast<uint32_t>(MOTOR1), 8);
    EXPECT_FALSE(handler.parseMotorMessage(CanBus::CAN_BUS1, msg));
}

TEST_F(DjiMotorSimHandlerTest, parseMotorMessage_single_sim_registered_low_mid)
{
    modm::can::Message msgLow(
        DjiMotorTxHandler::CAN_DJI_LOW_IDENTIFIER,
        8,
        0xffff'ffff'ffff'ffff,
        false);
    modm::can::Message msgHigh(
        DjiMotorTxHandler::CAN_DJI_HIGH_IDENTIFIER,
        8,
        0xffff'ffff'ffff'ffff,
        false);
    std::shared_ptr<MotorSim> sim(new MotorSim(MotorSim::M3508_CONFIG));
    handler.registerSim(sim, std::tuple<CanBus, MotorId>(CanBus::CAN_BUS1, MOTOR1));

    EXPECT_EQ(0, sim->getCurrent());

    EXPECT_FALSE(handler.parseMotorMessage(CanBus::CAN_BUS1, msgHigh));
    EXPECT_EQ(0, sim->getCurrent());

    EXPECT_TRUE(handler.parseMotorMessage(CanBus::CAN_BUS1, msgLow));
    EXPECT_NE(0, sim->getCurrent());
}

TEST_F(DjiMotorSimHandlerTest, parseMotorMessage_single_sim_registered_high_mid_can2)
{
    modm::can::Message msgLow(
        DjiMotorTxHandler::CAN_DJI_LOW_IDENTIFIER,
        8,
        0xffff'ffff'ffff'ffff,
        false);
    modm::can::Message msgHigh(
        DjiMotorTxHandler::CAN_DJI_HIGH_IDENTIFIER,
        8,
        0xffff'ffff'ffff'ffff,
        false);
    std::shared_ptr<MotorSim> sim(new MotorSim(MotorSim::M3508_CONFIG));
    handler.registerSim(sim, std::tuple<CanBus, MotorId>(CanBus::CAN_BUS2, MOTOR8));

    EXPECT_EQ(0, sim->getCurrent());

    EXPECT_FALSE(handler.parseMotorMessage(CanBus::CAN_BUS1, msgHigh));
    EXPECT_EQ(0, sim->getCurrent());

    EXPECT_FALSE(handler.parseMotorMessage(CanBus::CAN_BUS2, msgLow));
    EXPECT_EQ(0, sim->getCurrent());

    EXPECT_TRUE(handler.parseMotorMessage(CanBus::CAN_BUS2, msgHigh));
    EXPECT_NE(0, sim->getCurrent());
}

TEST_F(DjiMotorSimHandlerTest, encodeMessage_nullptr_msg_return_false)
{
    EXPECT_FALSE(handler.encodeMessage(CanBus::CAN_BUS1, nullptr));
}

TEST_F(DjiMotorSimHandlerTest, encodeMessage_nothing_registered_return_false)
{
    modm::can::Message msg(static_cast<uint32_t>(MOTOR1), 8);
    EXPECT_FALSE(handler.encodeMessage(CanBus::CAN_BUS1, &msg));
}

TEST_F(DjiMotorSimHandlerTest, encodeMessage_can1_message_only_can2_motors_registered_returns_false)
{
    modm::can::Message msg(static_cast<uint32_t>(MOTOR1), 8);
    std::shared_ptr<MotorSim> sim(new MotorSim(MotorSim::M3508_CONFIG));
    handler.registerSim(sim, std::tuple<CanBus, MotorId>(CanBus::CAN_BUS2, MOTOR1));

    EXPECT_FALSE(handler.encodeMessage(CanBus::CAN_BUS1, &msg));
}

TEST_F(DjiMotorSimHandlerTest, encodeMessage_encodes_motor_data)
{
    modm::can::Message msg(0, 8);

    std::shared_ptr<MotorSim> sim(new MotorSim(MotorSim::M3508_CONFIG));
    sim->setMotorInput(1'000);
    clock.time += 1'000;
    sim->update();

    handler.registerSim(sim, std::tuple<CanBus, MotorId>(CanBus::CAN_BUS1, MOTOR4));

    EXPECT_TRUE(handler.encodeMessage(CanBus::CAN_BUS1, &msg));

    int16_t reportedRPM =
        (static_cast<int16_t>(msg.data[2]) << 8) | (static_cast<int16_t>(msg.data[3]) & 0xff);

    EXPECT_EQ(static_cast<uint32_t>(MOTOR4), msg.identifier);
    EXPECT_EQ(sim->getRPM(), reportedRPM);
}
