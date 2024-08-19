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

#include "tap/communication/serial/terminal_serial.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/terminal_serial_callback_interface_mock.hpp"

using namespace testing;
using namespace tap::stub;
using namespace tap::mock;
using namespace tap::communication::serial;

TEST(TerminalSerial, update__with_no_handlers_added_always_prints_usage)
{
    tap::Drivers drivers;
    TerminalSerial serial(&drivers);

    char input[] = "foo\n";
    std::vector<char> inputVec(input, input + sizeof(input));
    serial.device.emplaceItemsInReadBuffer(inputVec);

    for (size_t i = 0; i < inputVec.size(); i++)
    {
        serial.update();
    }

    std::string output = serial.device.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("Header 'foo' not found"));
    EXPECT_THAT(output, HasSubstr("Usage"));

    char blankInput[] = "    \n   \r   ";
    inputVec = std::vector<char>(blankInput, blankInput + sizeof(blankInput));
    serial.device.emplaceItemsInReadBuffer(inputVec);

    for (size_t i = 0; i < inputVec.size(); i++)
    {
        serial.update();
    }

    EXPECT_EQ(0, serial.device.readAllItemsFromWriteBufferToString().size());
}

TEST(TerminalSerial, update__with_some_header_prints_usage_and_header_name_when_input_incorrect)
{
    tap::Drivers drivers;
    TerminalSerial serial(&drivers);

    TerminalSerialCallbackInterfaceMock interface;

    serial.addHeader("foo", &interface);

    char wrongInput[] = "bar\n";
    std::vector<char> inputVec = std::vector<char>(wrongInput, wrongInput + sizeof(wrongInput));
    serial.device.emplaceItemsInReadBuffer(inputVec);

    for (size_t i = 0; i < inputVec.size(); i++)
    {
        serial.update();
    }

    std::string output = serial.device.readAllItemsFromWriteBufferToString();
    EXPECT_THAT(output, HasSubstr("Header 'bar' not found"));
    EXPECT_THAT(output, HasSubstr("Usage"));
    EXPECT_THAT(output, HasSubstr("bar"));
}

TEST(TerminalSerial, update__with_some_header_calls_callback_when_header_valid)
{
    tap::Drivers drivers;
    TerminalSerial serial(&drivers);

    TerminalSerialCallbackInterfaceMock interface;

    EXPECT_CALL(interface, terminalSerialCallback)
        .WillOnce(
            [](char *, modm::IOStream &outputStream, bool streamingEnabled)
            {
                EXPECT_FALSE(streamingEnabled);

                outputStream << "hello world";
                return true;
            });

    serial.addHeader("foo", &interface);

    char correctInput[] = "foo\n";
    std::vector<char> inputVec =
        std::vector<char>(correctInput, correctInput + sizeof(correctInput));
    serial.device.emplaceItemsInReadBuffer(inputVec);

    for (size_t i = 0; i < inputVec.size(); i++)
    {
        serial.update();
    }

    std::string output = serial.device.readAllItemsFromWriteBufferToString();
    EXPECT_EQ("hello world", output);
}

TEST(TerminalSerial, update__with_some_header_calls_callback_passes_stream_enable_flag)
{
    tap::Drivers drivers;
    TerminalSerial serial(&drivers);

    TerminalSerialCallbackInterfaceMock interface;

    EXPECT_CALL(interface, terminalSerialCallback)
        .WillOnce(
            [](char *inputLine, modm::IOStream &outputStream, bool streamingEnabled)
            {
                EXPECT_TRUE(streamingEnabled);
                // the -S shouldn't be passed, instead it is passed via `streamingEnabled`
                EXPECT_THAT(inputLine, Not(HasSubstr("-S")));

                outputStream << "hello world";
                return true;
            });

    serial.addHeader("foo", &interface);

    char input[] = "foo -S\n";
    std::vector<char> inputVec = std::vector<char>(input, input + sizeof(input));
    serial.device.emplaceItemsInReadBuffer(inputVec);

    for (size_t i = 0; i < inputVec.size(); i++)
    {
        serial.update();
    }

    std::string output = serial.device.readAllItemsFromWriteBufferToString();
    EXPECT_EQ("hello world", output);
}

TEST(
    TerminalSerial,
    update__with_some_header_enters_streaming_mode_if_streaming_enabled_and_exits_correctly)
{
    tap::arch::clock::ClockStub clock;
    tap::Drivers drivers;
    TerminalSerial serial(&drivers);

    TerminalSerialCallbackInterfaceMock interface;

    EXPECT_CALL(interface, terminalSerialCallback)
        .WillOnce(
            [](char *, modm::IOStream &, bool streamingEnabled)
            {
                EXPECT_TRUE(streamingEnabled);
                return true;
            });

    EXPECT_CALL(interface, terminalSerialStreamCallback)
        .Times(2)
        .WillRepeatedly([](modm::IOStream &outputStream) { outputStream << "hi"; });

    serial.addHeader("foo", &interface);

    char input[] = "foo -S\n";
    std::vector<char> inputVec = std::vector<char>(input, input + sizeof(input) - 1);
    serial.device.emplaceItemsInReadBuffer(inputVec);

    for (size_t i = 0; i < inputVec.size(); i++)
    {
        serial.update();
    }

    serial.update();
    clock.time = TerminalSerial::STREAMING_PERIOD + 1;
    serial.update();
    clock.time = tap::arch::clock::getTimeMilliseconds() + TerminalSerial::STREAMING_PERIOD + 1;
    serial.update();

    std::string output = serial.device.readAllItemsFromWriteBufferToString();
}

TEST(TerminalSerial, update__with_some_header_doesnt_enter_streaming_mode_if_callback_return_false)
{
    tap::arch::clock::ClockStub clock;
    tap::Drivers drivers;
    TerminalSerial serial(&drivers);

    TerminalSerialCallbackInterfaceMock interface;

    EXPECT_CALL(interface, terminalSerialCallback)
        .WillOnce(
            [](char *, modm::IOStream &, bool streamingEnabled)
            {
                EXPECT_TRUE(streamingEnabled);
                return false;
            });

    EXPECT_CALL(interface, terminalSerialStreamCallback).Times(0);

    serial.addHeader("foo", &interface);

    char input[] = "foo -S\n";
    std::vector<char> inputVec = std::vector<char>(input, input + sizeof(input) - 1);
    serial.device.emplaceItemsInReadBuffer(inputVec);

    for (size_t i = 0; i < inputVec.size(); i++)
    {
        serial.update();
    }

    serial.update();
    clock.time = TerminalSerial::STREAMING_PERIOD + 1;
    serial.update();
    clock.time = TerminalSerial::STREAMING_PERIOD + 1;
    serial.update();
}
