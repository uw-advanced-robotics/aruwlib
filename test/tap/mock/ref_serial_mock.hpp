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

#ifndef REF_SERIAL_MOCK_HPP_
#define REF_SERIAL_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

namespace tap
{
namespace mock
{
class RefSerialMock : public serial::RefSerial
{
public:
    RefSerialMock(Drivers* drivers);
    virtual ~RefSerialMock();

    MOCK_METHOD(
        void,
        messageReceiveCallback,
        (const tap::serial::DJISerial::SerialMessage&),
        (override));
    MOCK_METHOD(bool, getRefSerialReceivingData, (), (const override));
    MOCK_METHOD(const Rx::RobotData&, getRobotData, (), (const override));
    MOCK_METHOD(const Rx::GameData&, getGameData, (), (const override));
    MOCK_METHOD(void, deleteGraphicLayer, (Tx::DeleteGraphicOperation, uint8_t), (override));
    MOCK_METHOD(void, sendGraphic, (Tx::Graphic1Message*, bool, bool), (override));
    MOCK_METHOD(void, sendGraphic, (Tx::Graphic2Message*, bool, bool), (override));
    MOCK_METHOD(void, sendGraphic, (Tx::Graphic5Message*, bool, bool), (override));
    MOCK_METHOD(void, sendGraphic, (Tx::Graphic7Message*, bool, bool), (override));
    MOCK_METHOD(void, sendGraphic, (Tx::GraphicCharacterMessage*, bool, bool), (override));
    MOCK_METHOD(
        void,
        configRobotToRobotMsgHeader,
        (Tx::RobotToRobotMessage*, uint16_t, RobotId, uint16_t),
        (override));
    MOCK_METHOD(RobotId, getRobotIdBasedOnCurrentRobotTeam, (RobotId), (override));
    MOCK_METHOD(
        void,
        attachRobotToRobotMessageHandler,
        (uint16_t, RobotToRobotMessageHandler*),
        (override));
};  // class RefSerialMock
}  // namespace mock
}  // namespace tap

#endif  // REF_SERIAL_MOCK_HPP_
