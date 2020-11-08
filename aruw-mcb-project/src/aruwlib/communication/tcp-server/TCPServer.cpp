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

#include <exception>
#include <iostream>
using std::cerr;
/**
 * TCP Server class to allow MCB simulator to communicate with stuff.
 * I actually have no idea what's happening ~ Tenzin
 */
namespace aruwlib
{
namespace communication
{
TCPServer::TCPServer(uint16_t portNumber)
    : socketOpened(false),
      clientConnected(false),
      listenFileDescriptor(-1),
      clientFileDescriptor(-1),
      serverPortNumber(portNumber)
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

TCPServer::~TCPServer()
{
    if (socketOpened)
    {
        close(listenFileDescriptor);
    }

    if (clientConnected)
    {
        close(clientFileDescriptor);
    }
}

/**
 * Accept a new client socket connection. Store file descriptor in
 * the "clientFileDescriptor" field.
 */
void TCPServer::acceptConnection()
{
    if (clientConnected)
    {
        close(clientFileDescriptor);
    }

    clientConnected = false;
    sockaddr_in clientAddress;
    socklen_t clientAddressLength = sizeof(clientAddress);
    clientFileDescriptor = accept(
        listenFileDescriptor,
        reinterpret_cast<sockaddr *>(&clientAddress),
        &clientAddressLength);
    if (clientFileDescriptor < 0)
    {
        perror("TCPServer failed to accept client connection");
        throw std::runtime_error("AcceptError");
    }
    else
    {
        clientConnected = true;
    }
}

/**
 * Post: Reads a message to the given "readBuffer" ensuring that messageLength bytes are
 * read.
 * 
 * Throws: runtime_error if read() fails.
 */
void TCPServer::readMessage(unsigned char* readBuffer, uint16_t messageLength)
{
    if (!clientConnected)
    {
        cerr << "Not connected to a client";
        return;
    }
    else
    {
        readBuffer[messageLength] = '\0';  // Null terminate the message
        uint16_t bytesRead = read(clientFileDescriptor, readBuffer, messageLength);
        if (bytesRead < 0) {
            perror("TCPServer failed to read from client");
            throw std::runtime_error("ReadingError");
        }
        while (bytesRead < messageLength)
        {
            bytesRead += read(clientFileDescriptor, readBuffer + bytesRead, messageLength - bytesRead);
        }
    }
}

/**
 * Write to connected ClientFileDescriptor, ensures that all bytes are sent.
 */
void TCPServer::writeToClient(const unsigned char *message, uint16_t bytes)
{
    uint16_t bytesWritten = write(clientFileDescriptor, message, bytes);
    if (bytesWritten < 0) {
        perror("TCPServer failed to write");
        throw std::runtime_error("WriteError");
    }
    while (bytesWritten < bytes)
    {
        bytesWritten += write(clientFileDescriptor, message + bytesWritten, bytes - bytesWritten);
    }
}

/**
 * Post: Returns the port number of this server.
 */
uint16_t TCPServer::getPortNumber() { return this->serverPortNumber; }

int32_t TCPServer::readInt32() {
    unsigned char buffer[4];
    readMessage(buffer, 4);
    int32_t answer = 0;
    for (char i = 0; i < 4; i++) {
        answer = answer & buffer[i];
        answer <<= 8;
    }
    return answer;
}

}  // namespace communication

}  // namespace aruwlib

#endif  // PLATFORM_HOSTED
