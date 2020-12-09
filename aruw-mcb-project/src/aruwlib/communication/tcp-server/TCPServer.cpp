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
#include "JSONMessages.hpp"

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
 * TCPServer constructor. Runs a server on the given portnumber.
 * Also takes a function pointer. This function is left to the implementer
 * of a specific TCP server to define (it should take an int16_t parameter
 * for client file descriptor). Ideally mainLoop should use a non-blocking
 * loop as currently since threads detach its possible that if a client never
 * responds there could be threads that never close.
 */
int16_t TCPServer::listenFileDescriptor = -1;
int16_t TCPServer::mainClientDescriptor = -1;
TCPServer::TCPServer()
    : socketOpened(false),
      clientConnected(false),
      serverAddress()
{
    // Do sockety stuff.
    listenFileDescriptor = socket(AF_INET, SOCK_STREAM, 0);
    if (listenFileDescriptor < 0)
    {
        perror("TCPServer failed to open socket");
        throw std::runtime_error("SocketError");
    }

    socketOpened = true;
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
    std::thread acceptThread(&TCPServer::getConnection, this);
    acceptThread.detach();
    cerr << "TCPServer initialized, acceptThread begun" << std::endl;
}

TCPServer::~TCPServer() {
    close(listenFileDescriptor); 
    close(mainClientDescriptor);    
}

void TCPServer::getConnection()
{
    sockaddr_in clientAddress;
    socklen_t clientAddressLength = sizeof(clientAddress);
    int16_t clientFileDescriptor;
    mainClientDescriptor = accept(listenFileDescriptor, 
                                reinterpret_cast<sockaddr *>(&clientAddress), 
                                &clientAddressLength);
    cerr << "TCPServer: connection accepted";
}

/**
 * Post: Returns the port number of this server.
 */
uint16_t TCPServer::getPortNumber() { return this->serverPortNumber; }

/**
 * Writes "messageLength" bytes of "message" to the mainClient
 */
void TCPServer::writeToClient(const char* message, int32_t messageLength) {
    if (mainClientDescriptor < 0) {
        // Not necessarily an error if fileDescriptor still hasn't been opened
        // so we just don't write to anything and early return.
        cerr << "TCPServer: mainClientDescriptor not connected yet" << std::endl;
        return;
    }
    writeMessage(mainClientDescriptor, message, messageLength);
}

/**
 * Pre: Message readBuffer must be messageLength + 1 bytes long otherwise out of bounds
 * array access will occur and behavior will be undefined.
 * Post: Reads a message to the given "readBuffer" ensuring that messageLength bytes are
 * read.
 * 
 * Throws: runtime_error if read() fails.
 */
void readMessage(int16_t fileDescriptor, char* readBuffer, uint16_t messageLength)
{
    readBuffer[messageLength] = '\0';  // Null terminate the message
    uint16_t bytesRead = read(fileDescriptor, readBuffer, messageLength);
    while (bytesRead < messageLength)
    {
        // uint can't hold negative value dumbass
        int32_t n = read(fileDescriptor, readBuffer + bytesRead, messageLength - bytesRead);
        if (n < 0) {
            if (errno == EAGAIN or errno == EINTR) {
                continue;
            } else {
                perror("TCPServer failed to read from client");
                throw std::runtime_error("ReadingError");
            }
        }
        bytesRead += n;
    }
}

/**
 * Write to connected connectionFileDescriptor, ensures that all bytes are sent.
 */
void writeMessage(int16_t fileDescriptor, const char *message, uint16_t bytes)
{
    uint32_t bytesWritten = 0;
    while (bytesWritten < bytes)
    {
        int32_t n = write(fileDescriptor, message + bytesWritten, bytes - bytesWritten);
        // Error handling's completely wrong dumbass.
        if (n < 0) {
            if (errno == EAGAIN or errno == EINTR) {
                std::cout << "EAGAIN/EINTR occurred" << std::endl;
                continue;
            } else {
                perror("TCPServer failed to write");
                throw std::runtime_error("WriteError");
            }
        }
        bytesWritten += n;
    }
}

/**
 * Read the next 4 bytes as a big endian int32_t
 */
int32_t readInt32(int16_t fileDescriptor) {
    char buffer[4];
    readMessage(fileDescriptor, buffer, 4);
    int32_t answer = 0;
    for (char i = 0; i < 4; i++) {
        answer = answer | buffer[i];
        answer <<= 8;
    }
    return answer;
}

// Definition of static variable. mainServer never created otherwise.
TCPServer TCPServer::mainServer;

}  // namespace communication

}  // namespace aruwlib

#endif  // PLATFORM_HOSTED
