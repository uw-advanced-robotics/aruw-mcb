/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifdef PLATFORM_HOSTED

#include "TCPServer.hpp"

#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <exception>
#include <iostream>
#include <thread>
using std::cerr;
/**
 * TCP Server class to allow MCB simulator to communicate with stuff.
 * I actually have no idea what's happening ~ Tenzin
 */
namespace aruwlib
{
namespace communication
{
/**
 * Post: Reads a message to the given "readBuffer" ensuring that messageLength bytes are
 * read.
 *
 * Throws: runtime_error if read() fails.
 */
void readMessage(int16_t fileDescriptor, char *readBuffer, uint16_t messageLength)
{
    readBuffer[messageLength] = '\0';  // Null terminate the message
    uint16_t bytesRead = read(fileDescriptor, readBuffer, messageLength);
    while (bytesRead < messageLength)
    {
        if (bytesRead < 0)
        {
            if (errno == EAGAIN or errno == EINTR)
            {
                continue;
            }
            else
            {
                perror("TCPServer failed to read from client");
                throw std::runtime_error("ReadingError");
            }
        }
        bytesRead += read(fileDescriptor, readBuffer + bytesRead, messageLength - bytesRead);
    }
}

/**
 * Write to connected connectionFileDescriptor, ensures that all bytes are sent.
 */
void writeMessage(int16_t fileDescriptor, const char *message, uint16_t bytes)
{
    uint16_t bytesWritten = write(fileDescriptor, message, bytes);
    while (bytesWritten < bytes)
    {
        if (bytesWritten < 0)
        {
            if (errno == EAGAIN or errno == EINTR)
            {
                continue;
            }
            else
            {
                perror("TCPServer failed to write");
                throw std::runtime_error("WriteError");
            }
        }
        bytesWritten += write(fileDescriptor, message + bytesWritten, bytes - bytesWritten);
    }
}

/**
 * Read the next 4 bytes as a big endian int32_t
 */
int32_t readInt32(int16_t fileDescriptor)
{
    char buffer[4];
    readMessage(fileDescriptor, buffer, 4);
    int32_t answer = 0;
    for (char i = 0; i < 4; i++)
    {
        answer = answer | buffer[i];
        answer <<= 8;
    }
    return answer;
}

/**
 * TCPServer constructor. Runs a server using the given portnumber.
 * Also takes a function pointer. This function is left to the implementer
 * of a specific TCP server to define (it should take an int16_t parameter
 * for client file descriptor). Ideally mainLoop should use a non-blocking
 * loop as currently since threads detach its possible that if a client never
 * responds there could be threads that never close.
 */
TCPServer::TCPServer(uint16_t portNumber)
    : socketOpened(false),
      clientConnected(false),
      listenFileDescriptor(-1),
      serverPortNumber(portNumber),
      running_(false)
{
    // Do sockety stuff.
    listenFileDescriptor = socket(AF_INET, SOCK_STREAM, 0);
    if (listenFileDescriptor < 0)
    {
        perror("TCPServer failed to open socket");
        throw std::runtime_error("SocketError");
    }

    socketOpened = true;
    memset(reinterpret_cast<char *>(&serverAddress), 0, sizeof(serverAddress));  // Initialize
    // bytes of serverAddress to 0.
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(serverPortNumber);
    serverAddress.sin_addr.s_addr = INADDR_ANY;
    if (bind(
            listenFileDescriptor,
            reinterpret_cast<sockaddr *>(&serverAddress),
            sizeof(serverAddress)) < 0)
    {
        perror("TCPServer failed to bind socket");
        throw std::runtime_error("BindError");
    }

    listen(listenFileDescriptor, LISTEN_QUEUE_SIZE);
}

TCPServer::~TCPServer() { close(listenFileDescriptor); }

void TCPServer::start()
{
    if (running_)
    {
        throw new std::runtime_error("TCPServer: Cannot start() when already running");
    }
    running_ = true;
    std::thread listenLoop(&TCPServer::run, this);
    listenLoop.detach();
}

void TCPServer::stop() { running_ = false; }

void TCPServer::run()
{
    sockaddr_in clientAddress;
    socklen_t clientAddressLength = sizeof(clientAddress);
    int16_t clientFileDescriptor;
    while (running_ and (clientFileDescriptor = accept(
                             listenFileDescriptor,
                             reinterpret_cast<sockaddr *>(&clientAddress),
                             &clientAddressLength)))
    {
        std::thread clientThread(&TCPServer::clientLoop, this, clientFileDescriptor);
        clientThread.detach();
        std::cout << "Connection accepted" << std::endl;
    }
    cerr << "exited run() loop";
}

void TCPServer::clientLoop(int16_t clientFileDescriptor)
{
    try
    {
        (&TCPServer::mainLoop);
    }
    catch (std::runtime_error e)
    {
        // empty catch block, regardless what happened we just want to close fileDescriptor
    }
    close(clientFileDescriptor);
}

/**
 * Post: Returns the port number of this server.
 */
uint16_t TCPServer::getPortNumber() { return this->serverPortNumber; }

}  // namespace communication

}  // namespace aruwlib

#endif  // PLATFORM_HOSTED
