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

#include <threads.h>

#include <string>

#include <gtest/gtest.h>

#include "tap/communication/tcp-server/tcp_server.hpp"

#include "tcp_test_client.hpp"

using namespace tap::communication;
using test::communication::TCPClient;

TEST(TCPServerTests, SendingCorrectMessages)
{
    pid_t finished_child;
    pid_t child_pid;
    int status = 0;
    char response[32];
    memset(response, 0, sizeof(response));

    TCPServer tcpServer(8889);
    int16_t serverPort = tcpServer.getPortNumber();
    // Fork since calls to accept() and connect() are blocking
    // so we need some form of multi-threading to be able to test
    // TCPServer.
    if ((child_pid = fork()) == 0)
    {
        tcpServer.getConnection();
        tcpServer.writeToClient("Test message 1 2 3", 18);
        tcpServer.closeConnection();
        exit(0);
    }
    else
    {
        TCPClient client("localhost", serverPort);
        client.Read(response, 18);
    }

    // Wait for all children to exit
    while ((finished_child = wait(&status)) > 0);
    EXPECT_STREQ(response, "Test message 1 2 3");
}
