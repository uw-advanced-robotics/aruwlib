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

#include <memory>

#include <gtest/gtest.h>

#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/mock/can_rx_handler_mock.hpp"
#include "tap/mock/can_rx_listener_mock.hpp"

using namespace testing;
using namespace std;
using namespace tap::mock;

class CanRxHandlerTest : public Test
{
protected:
    CanRxHandlerTest() : handler(&drivers) {}

    void constructListeners(tap::can::CanBus canBus = tap::can::CanBus::CAN_BUS1)
    {
        for (uint32_t i = tap::motor::MOTOR1; i <= tap::motor::MOTOR8; i++)
        {
            auto listener = make_unique<CanRxListenerMock>(&drivers, i, canBus);
            listeners.push_back(move(listener));
        }
    }

    tap::Drivers drivers;
    tap::can::CanRxHandler handler;
    vector<unique_ptr<CanRxListenerMock>> listeners;
};

TEST(CanRxHandler, ListenerAttachesSelf)
{
    tap::Drivers drivers;
    CanRxListenerMock listener(&drivers, 0, tap::can::CanBus::CAN_BUS1);

    EXPECT_CALL(drivers.canRxHandler, attachReceiveHandler(&listener));
    EXPECT_CALL(drivers.canRxHandler, removeReceiveHandler(testing::Ref(listener)));

    listener.attachSelfToRxHandler();
}

TEST_F(CanRxHandlerTest, attachReceiveHandler_attaches_listener_can2)
{
    listeners.push_back(unique_ptr<CanRxListenerMock>(
        new CanRxListenerMock(&drivers, tap::motor::MOTOR1, tap::can::CanBus::CAN_BUS2)));

    handler.attachReceiveHandler(listeners[0].get());

    EXPECT_EQ(
        listeners[0].get(),
        handler.getHandlerStore(
            tap::can::CanBus::CAN_BUS2)[tap::can::CanRxHandler::lookupTableIndexForCanId(
            listeners[0]->canIdentifier)]);

    handler.removeReceiveHandler(*listeners[0]);
}

TEST_F(CanRxHandlerTest, ListenerAttachesAndDetatchesInArray)
{
    constructListeners();

    for (auto &listener : listeners)
    {
        handler.attachReceiveHandler(listener.get());

        int normalizedId =
            tap::can::CanRxHandler::lookupTableIndexForCanId(listener->canIdentifier);

        EXPECT_EQ(
            listener.get(),
            handler.getHandlerStore(tap::can::CanBus::CAN_BUS1)[normalizedId]);

        handler.removeReceiveHandler(*listener);

        EXPECT_EQ(nullptr, handler.getHandlerStore(tap::can::CanBus::CAN_BUS1)[normalizedId]);
    }
}

TEST_F(CanRxHandlerTest, MessageIsProcessedByCorrectListener)
{
    constructListeners();

    for (auto &listener : listeners)
    {
        EXPECT_CALL(*listener, processMessage);
    }

    for (auto &listener : listeners)
    {
        handler.attachReceiveHandler(listener.get());

        const modm::can::Message rxMessage(listener->canIdentifier);

        handler.processReceivedCanData(
            rxMessage,
            handler.getHandlerStore(tap::can::CanBus::CAN_BUS1));
    }
}

TEST_F(CanRxHandlerTest, ErrorIsThrownWithOOBMessageID)
{
    const modm::can::Message rxMessage(9);

    EXPECT_CALL(drivers.errorController, addToErrorList);

    handler.processReceivedCanData(rxMessage, handler.getHandlerStore(tap::can::CanBus::CAN_BUS1));
}

TEST_F(CanRxHandlerTest, removeReceiveHandler__error_logged_with_oob_can_rx_listener_id)
{
    CanRxListenerMock canRxListenerHi(&drivers, 0xffff, tap::can::CanBus::CAN_BUS1);
    CanRxListenerMock canRxListenerLo(&drivers, 0x0, tap::can::CanBus::CAN_BUS1);

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(2);

    handler.removeReceiveHandler(canRxListenerHi);
    handler.removeReceiveHandler(canRxListenerLo);
}

TEST_F(CanRxHandlerTest, pollCanData_can1_calls_process_message_passing_msg_to_correct_listener)
{
    constructListeners();

    handler.attachReceiveHandler(listeners[0].get());

    modm::can::Message msg(tap::motor::MOTOR1, 8, 0xffff'ffff'ffff'ffff, false);

    ON_CALL(drivers.can, getMessage(tap::can::CanBus::CAN_BUS1, _))
        .WillByDefault([&](tap::can::CanBus, modm::can::Message *message) {
            *message = msg;
            return true;
        });
    ON_CALL(drivers.can, getMessage(tap::can::CanBus::CAN_BUS2, _))
        .WillByDefault([&](tap::can::CanBus, modm::can::Message *) { return false; });

    EXPECT_CALL(*listeners[0], processMessage);

    handler.pollCanData();
}

TEST_F(CanRxHandlerTest, pollCanData_can2_calls_process_message_passing_msg_to_correct_listener)
{
    constructListeners(tap::can::CanBus::CAN_BUS2);

    handler.attachReceiveHandler(listeners[0].get());

    modm::can::Message msg(tap::motor::MOTOR1, 8, 0xffff'ffff'ffff'ffff, false);

    ON_CALL(drivers.can, getMessage(tap::can::CanBus::CAN_BUS1, _))
        .WillByDefault([&](tap::can::CanBus, modm::can::Message *) { return false; });
    ON_CALL(drivers.can, getMessage(tap::can::CanBus::CAN_BUS2, _))
        .WillByDefault([&](tap::can::CanBus, modm::can::Message *message) {
            *message = msg;
            return true;
        });

    EXPECT_CALL(*listeners[0], processMessage);

    handler.pollCanData();
}
