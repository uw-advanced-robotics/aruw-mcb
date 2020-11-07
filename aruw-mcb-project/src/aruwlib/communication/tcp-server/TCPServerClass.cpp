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

#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <exception>
#include <iostream>
#include "TCPServerClass.hpp"
using std::cerr;
/**
 * TCP Server class to allow MCB simulator to communicate with stuff.
 * I actually have no idea what's happening ~ Tenzin 
 */
namespace aruwlib
{
namespace communication
{
TCPServer::TCPServer(uint16_t portNumber) : socketOpened(false), clientConnected(false)
{
    clientFileDescriptor = -1;

    // Do sockety stuff.
    serverPortNumber = portNumber;
    listenFileDescriptor = socket(AF_INET, SOCK_STREAM, 0);
    if (listenFileDescriptor < 0)
    {
        perror("ERROR opening socket");
        throw "ServerCreationFailed";
    }
    socketOpened = true;
    memset((char *)&serverAddress, 0, sizeof(serverAddress));  // Initialize
    // bytes of serverAddress to 0.
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(serverPortNumber);
    serverAddress.sin_addr.s_addr = INADDR_ANY;
    if (bind(listenFileDescriptor, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0)
    {
        perror("ERROR binding socket");
        throw "ServerCreationFailed";
    }
    listen(listenFileDescriptor, 5);
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
    clientFileDescriptor =
        accept(listenFileDescriptor, (sockaddr *)&clientAddress, &clientAddressLength);
    if (clientFileDescriptor < 0)
    {
        cerr << "ERROR on accept";
        throw "ClientConnectionFailed";
    }
    else
    {
        clientConnected = true;
    }
}

/**
 * Post: Reads a message to the class's buffer ensuring that MESSAGE_LENGTH bytes are
 * read, before finally returning a pointer to the beginning of the buffer. 
 */
const unsigned char *TCPServer::readMessage()
{
    if (!clientConnected)
    {
        cerr << "Not connected to a client";
        return nullptr;
    }
    else
    {
        buffer[MESSAGE_LENGTH] = '\0'; // Null terminate the message
        uint16_t bytesRead = read(clientFileDescriptor, buffer, MESSAGE_LENGTH);
        while (bytesRead < MESSAGE_LENGTH)
        {
            bytesRead +=
                read(clientFileDescriptor, buffer + bytesRead, MESSAGE_LENGTH - bytesRead);
        }
        return buffer;
    }
}

/**
 * Write to connected ClientFileDescriptor, ensures that all bytes are sent.
 */
void TCPServer::writeToClient(unsigned char *message, uint16_t bytes)
{
    uint16_t bytesWritten = write(clientFileDescriptor, message, bytes);
    while (bytesWritten < bytes)
    {
        bytesWritten += write(clientFileDescriptor, message + bytesWritten, bytes - bytesWritten);
    }
}

/**
 * Post: Returns the port number of this server. 
 */
uint16_t TCPServer::getPortNumber() { return this->serverPortNumber; }

}  // namespace communication

}  // namespace aruwlib

#endif  // PLATFORM_HOSTED
