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

#include "tap/algorithms/crc.hpp"
#include "tap/architecture/endianness_wrappers.hpp"
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/drivers.hpp"

using namespace tap::communication::serial;
using namespace tap::arch;
using namespace testing;
using namespace tap;
using namespace tap::algorithms;

class DJISerialTester : public DJISerial
{
public:
    DJISerialTester(Drivers *drivers, Uart::UartPort port, bool isRxCRCEnforcementEnabled)
        : DJISerial(drivers, port, isRxCRCEnforcementEnabled)
    {
    }

    void messageReceiveCallback(const ReceivedSerialMessage &completeMessage) override
    {
        lastMsg = completeMessage;
    }

    ReceivedSerialMessage lastMsg;
};

TEST(DJISerial, updateSerial_parseMessage_single_byte_at_a_time_crcenforcement)
{
    Drivers drivers;
    DJISerialTester serial(&drivers, Uart::Uart1, true);

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(0);

    uint8_t rawMessage[19];
    uint16_t currByte = 0;

    convertToLittleEndian(static_cast<uint8_t>(0xa5), rawMessage);
    convertToLittleEndian(static_cast<uint16_t>(10), rawMessage + 1);
    convertToLittleEndian(static_cast<uint8_t>(123), rawMessage + 3);
    convertToLittleEndian(calculateCRC8(rawMessage, 4), rawMessage + 4);
    convertToLittleEndian(static_cast<uint16_t>(2), rawMessage + 5);
    for (uint8_t i = 0; i < 10; i++)
    {
        rawMessage[i + 7] = i;
    }
    convertToLittleEndian(calculateCRC16(rawMessage, 17), rawMessage + 17);

    ON_CALL(drivers.uart, read(Uart::Uart1, _, _))
        .WillByDefault(
            [&](Uart::UartPort, uint8_t *data, std::size_t length)
            {
                if (length == 0)
                {
                    return 0;
                }

                if (currByte >= sizeof(rawMessage))
                {
                    return 0;
                }
                *data = rawMessage[currByte];
                currByte++;
                return 1;
            });

    for (int i = 0; i < 100; i++)
    {
        serial.updateSerial();
    }

    EXPECT_EQ(0xa5, serial.lastMsg.header.headByte);
    EXPECT_EQ(10, serial.lastMsg.header.dataLength);
    EXPECT_EQ(123, serial.lastMsg.header.seq);
    EXPECT_EQ(calculateCRC8(rawMessage, 4), serial.lastMsg.header.CRC8);
    EXPECT_EQ(2, serial.lastMsg.messageType);

    for (uint8_t i = 0; i < 10; i++)
    {
        EXPECT_EQ(i, serial.lastMsg.data[i]);
    }

    EXPECT_EQ(calculateCRC16(rawMessage, 17), serial.lastMsg.CRC16);
}

TEST(DJISerial, updateSerial_parseMessage_crc8_one_off)
{
    Drivers drivers;
    DJISerialTester serial(&drivers, Uart::Uart1, true);

    EXPECT_CALL(drivers.errorController, addToErrorList)
        .WillOnce([&](const tap::errors::SystemError &error)
                  { EXPECT_TRUE(errorDescriptionContainsSubstr(error, "CRC8 failure")); });

    uint8_t rawMessage[19];
    uint16_t currByte = 0;

    convertToLittleEndian(static_cast<uint8_t>(0xa5), rawMessage);
    convertToLittleEndian(static_cast<uint16_t>(10), rawMessage + 1);
    convertToLittleEndian(static_cast<uint8_t>(123), rawMessage + 3);
    convertToLittleEndian(calculateCRC8(rawMessage, 4) - 1, rawMessage + 4);
    convertToLittleEndian(static_cast<uint16_t>(2), rawMessage + 5);
    for (uint8_t i = 0; i < 10; i++)
    {
        rawMessage[i + 7] = i;
    }
    convertToLittleEndian(calculateCRC16(rawMessage, 17), rawMessage + 17);

    ON_CALL(drivers.uart, read(Uart::Uart1, _, _))
        .WillByDefault(
            [&](Uart::UartPort, uint8_t *data, std::size_t length)
            {
                if (length == 0)
                {
                    return 0;
                }

                if (currByte >= sizeof(rawMessage))
                {
                    return 0;
                }
                *data = rawMessage[currByte];
                currByte++;
                return 1;
            });

    for (int i = 0; i < 100; i++)
    {
        serial.updateSerial();
    }
}

TEST(DJISerial, updateSerial_parseMessage_crc16_one_off)
{
    Drivers drivers;
    DJISerialTester serial(&drivers, Uart::Uart1, true);

    EXPECT_CALL(drivers.errorController, addToErrorList)
        .WillOnce([&](const tap::errors::SystemError &error)
                  { EXPECT_TRUE(errorDescriptionContainsSubstr(error, "CRC16 failure")); });

    uint8_t rawMessage[19];
    uint16_t currByte = 0;

    convertToLittleEndian(static_cast<uint8_t>(0xa5), rawMessage);
    convertToLittleEndian(static_cast<uint16_t>(10), rawMessage + 1);
    convertToLittleEndian(static_cast<uint8_t>(123), rawMessage + 3);
    convertToLittleEndian(calculateCRC8(rawMessage, 4), rawMessage + 4);
    convertToLittleEndian(static_cast<uint16_t>(2), rawMessage + 5);
    for (uint8_t i = 0; i < 10; i++)
    {
        rawMessage[i + 7] = i;
    }
    convertToLittleEndian<uint16_t>(calculateCRC16(rawMessage, 17) - 1, rawMessage + 17);

    ON_CALL(drivers.uart, read(Uart::Uart1, _, _))
        .WillByDefault(
            [&](Uart::UartPort, uint8_t *data, std::size_t length)
            {
                if (length == 0)
                {
                    return 0;
                }

                if (currByte >= sizeof(rawMessage))
                {
                    return 0;
                }
                *data = rawMessage[currByte];
                currByte++;
                return 1;
            });

    for (int i = 0; i < 100; i++)
    {
        serial.updateSerial();
    }
}

TEST(DJISerial, updateSerial_parseMessage_msg_length_0)
{
    Drivers drivers;
    DJISerialTester serial(&drivers, Uart::Uart1, true);

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(0);

    uint8_t rawMessage[9];
    uint16_t currByte = 0;

    convertToLittleEndian(static_cast<uint8_t>(0xa5), rawMessage);
    convertToLittleEndian(static_cast<uint16_t>(0), rawMessage + 1);
    convertToLittleEndian(static_cast<uint8_t>(123), rawMessage + 3);
    convertToLittleEndian(calculateCRC8(rawMessage, 4), rawMessage + 4);
    convertToLittleEndian(static_cast<uint16_t>(2), rawMessage + 5);
    convertToLittleEndian(calculateCRC16(rawMessage, 7), rawMessage + 7);

    ON_CALL(drivers.uart, read(Uart::Uart1, _, _))
        .WillByDefault(
            [&](Uart::UartPort, uint8_t *data, std::size_t length)
            {
                if (length == 0)
                {
                    return 0;
                }

                if (currByte >= sizeof(rawMessage))
                {
                    return 0;
                }
                *data = rawMessage[currByte];
                currByte++;
                return 1;
            });

    for (int i = 0; i < 100; i++)
    {
        serial.updateSerial();
    }
}

TEST(DJISerial, updateSerial_parseMessage_msg_length_too_big)
{
    Drivers drivers;
    DJISerialTester serial(&drivers, Uart::Uart1, false);

    EXPECT_CALL(drivers.errorController, addToErrorList)
        .WillOnce(
            [&](const tap::errors::SystemError &error)
            {
                EXPECT_TRUE(errorDescriptionContainsSubstr(
                    error,
                    "received message length longer than allowed max"));
            });

    uint8_t rawMessage[19];
    uint16_t currByte = 0;

    convertToLittleEndian(static_cast<uint8_t>(0xa5), rawMessage);
    convertToLittleEndian(static_cast<uint16_t>(0xffff), rawMessage + 1);
    convertToLittleEndian(static_cast<uint8_t>(123), rawMessage + 3);
    convertToLittleEndian(calculateCRC8(rawMessage, 4), rawMessage + 4);
    convertToLittleEndian(static_cast<uint16_t>(2), rawMessage + 5);
    for (uint8_t i = 0; i < 10; i++)
    {
        rawMessage[i + 7] = i;
    }
    convertToLittleEndian(calculateCRC16(rawMessage, 17), rawMessage + 17);

    ON_CALL(drivers.uart, read(Uart::Uart1, _, _))
        .WillByDefault(
            [&](Uart::UartPort, uint8_t *data, std::size_t length)
            {
                if (length == 0)
                {
                    return 0;
                }

                if (currByte >= sizeof(rawMessage))
                {
                    return 0;
                }
                *data = rawMessage[currByte];
                currByte++;
                return 1;
            });

    for (int i = 0; i < 100; i++)
    {
        serial.updateSerial();
    }
}

TEST(DJISerial, updateSerial_parseMessage_all_bytes_received_at_once_crcenforcement)
{
    Drivers drivers;
    DJISerialTester serial(&drivers, Uart::Uart1, true);

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(0);

    uint8_t rawMessage[19];
    uint16_t currByte = 0;

    convertToLittleEndian(static_cast<uint8_t>(0xa5), rawMessage);
    convertToLittleEndian(static_cast<uint16_t>(10), rawMessage + 1);
    convertToLittleEndian(static_cast<uint8_t>(123), rawMessage + 3);
    convertToLittleEndian(calculateCRC8(rawMessage, 4), rawMessage + 4);
    convertToLittleEndian(static_cast<uint16_t>(2), rawMessage + 5);
    for (uint8_t i = 0; i < 10; i++)
    {
        rawMessage[i + 7] = i;
    }
    convertToLittleEndian(calculateCRC16(rawMessage, 17), rawMessage + 17);

    ON_CALL(drivers.uart, read(Uart::Uart1, _, _))
        .WillByDefault(
            [&](Uart::UartPort, uint8_t *data, std::size_t length)
            {
                int bytesRead = 0;
                for (std::size_t i = 0; i < length; i++)
                {
                    if (currByte == sizeof(rawMessage))
                    {
                        return bytesRead;
                    }
                    else
                    {
                        data[i] = rawMessage[currByte];
                        currByte++;
                        bytesRead++;
                    }
                }
                return bytesRead;
            });

    for (int i = 0; i < 3; i++)
    {
        serial.updateSerial();
    }

    EXPECT_EQ(0xa5, serial.lastMsg.header.headByte);
    EXPECT_EQ(10, serial.lastMsg.header.dataLength);
    EXPECT_EQ(123, serial.lastMsg.header.seq);
    EXPECT_EQ(calculateCRC8(rawMessage, 4), serial.lastMsg.header.CRC8);
    EXPECT_EQ(2, serial.lastMsg.messageType);

    for (uint8_t i = 0; i < 10; i++)
    {
        EXPECT_EQ(i, serial.lastMsg.data[i]);
    }

    EXPECT_EQ(calculateCRC16(rawMessage, 17), serial.lastMsg.CRC16);
}
