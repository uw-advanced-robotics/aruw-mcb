/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <string>
#include <thread>

#include <aruwlib/communication/tcp-server/TCPServer.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "TCPTestClient.hpp"

using namespace aruwlib::communication;
using test::communication::TCPClient;

void sendingCorrectMessagesHelper(TCPServer* tcpServer, const char* message)
{
    tcpServer->getConnection();
    tcpServer->writeToClient(message, strlen(message));
    tcpServer->closeConnection();
}

TEST(TCPServerTests, SendingCorrectMessages)
{
    char response[32];
    memset(response, 0, sizeof(response));

    TCPServer tcpServer(8889);
    int16_t serverPort = tcpServer.getPortNumber();
    // Fork since calls to accept() and connect() are blocking
    // so we need some form of multi-threading to be able to test
    // TCPServer.
    std::thread serverThread(sendingCorrectMessagesHelper, &tcpServer, "Test message 1 2 3");

    TCPClient client("localhost", serverPort);
    client.Read(response, 18);

    serverThread.join();

    EXPECT_STREQ(response, "Test message 1 2 3");
}

void sendMessageFromClientToServer(const char* message, int port)
{
    TCPClient client("localhost", port);
    client.Write(message);
}

TEST(TCPServerTests, ReadingCorrectMessages)
{
    char readBuffer[32];
    memset(readBuffer, 0, sizeof(readBuffer));  // unnecessary, but sanity check
    const int serverPort = 8889;
    TCPServer tcpServer(serverPort);

    std::thread client(sendMessageFromClientToServer, "123abc", serverPort);
    tcpServer.getConnection();
    tcpServer.readFromClient(readBuffer, 6);

    client.join();

    tcpServer.closeConnection();
    EXPECT_STREQ(readBuffer, "123abc");
}

TEST(TCPServerTests, MessageReadyFlagsDefaultInitialized) 
{
    TCPServer tcpServer(8889);
    EXPECT_EQ(false, tcpServer.isMessageReady(REMOTE));
}
