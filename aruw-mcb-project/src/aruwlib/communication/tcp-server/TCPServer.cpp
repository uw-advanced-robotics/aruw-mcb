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
#include <iostream>
#include <stdexcept>

#include "JSONMessages.hpp"
using std::cerr;
/**
 * TCP Server class to allow MCB simulator to communicate with stuff.
 * I only have some idea of what's happening ~ Tenzin
 */
namespace aruwlib
{
namespace communication
{
TCPServer::TCPServer(int targetPortNumber)
    : socketOpened(false),
      clientConnected(false),
      mainClientDescriptor(-1),
      serverAddress(),
      portNumber(-1)
{
    // 0-initialize class arrays
    memset(messageReadyFlags, false, sizeof(messageReadyFlags));
    memset(remoteMessageBuffer, 0, sizeof(remoteMessageBuffer));

    // Do sockety stuff.
    listenFileDescriptor = socket(AF_INET, SOCK_STREAM, 0);
    if (listenFileDescriptor < 0)
    {
        perror("TCPServer failed to open socket");
        throw std::runtime_error("SocketError");
    }

    int yes = 1;
    if (setsockopt(listenFileDescriptor, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)))
    {
        perror("TCPSever failed to set socket options");
        throw std::runtime_error("SocketOptionsError");
    }

    socketOpened = true;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(targetPortNumber);
    serverAddress.sin_addr.s_addr = INADDR_ANY;

    int n = bind(
        listenFileDescriptor,
        reinterpret_cast<sockaddr*>(&serverAddress),
        sizeof(serverAddress));

    if (n < 0)
    {
        perror("TCPServer failed to bind socket");
        // Good reference for why this error occurs and what it means:
        // https://hea-www.harvard.edu/~fine/Tech/addrinuse.html
        std::cerr << "If you're seeing this error it means that the TCPServer "
                     "has failed to bind to a port too many times. If you're seeing this "
                     "message, something is probably really messed up."
                  << std::endl;
        throw std::runtime_error("BindError");
    }

    portNumber = targetPortNumber;

    listen(listenFileDescriptor, LISTEN_QUEUE_SIZE);
    std::cout << "TCPServer initialized on port: " << targetPortNumber << std::endl;
    std::cout << "call getConnection() to accept client" << std::endl;
}

TCPServer::~TCPServer()
{
    if (listenFileDescriptor >= 0) {
        close(listenFileDescriptor);
    }
    if (mainClientDescriptor >= 0) {
        close(mainClientDescriptor);
    }
}

TCPServer* TCPServer::MainServer()
{
#ifdef ENV_UNIT_TESTS
    return nullptr;
#else
    return &mainServer;
#endif
}

void TCPServer::getConnection()
{
    sockaddr_in clientAddress;
    socklen_t clientAddressLength = sizeof(clientAddress);
    // Accept next connection in queue and create new non-blocking socket.
    mainClientDescriptor = accept4(
        listenFileDescriptor,
        reinterpret_cast<sockaddr*>(&clientAddress),
        &clientAddressLength,
        SOCK_NONBLOCK);
    if (mainClientDescriptor < 0)
    {
        perror("TCPServer failed to accept connection");
        throw std::runtime_error("AcceptError");
    }
    cerr << "TCPServer: connection accepted" << std::endl;
}

void TCPServer::closeConnection()
{
    close(mainClientDescriptor);
    mainClientDescriptor = -1;
    std::cout << "TCPServer: closed connection with client, "
                 "use getConnection() to connect to a new one\n";
}

uint16_t TCPServer::getPortNumber() { return this->portNumber; }

void TCPServer::writeToClient(const char* message, int32_t messageLength)
{
    if (mainClientDescriptor < 0)
    {
        // Not necessarily an error if fileDescriptor still hasn't been opened
        // so we just don't write to anything and early return.
        cerr << "TCPServer: mainClientDescriptor not connected yet" << std::endl;
        return;
    }
    try
    {
        writeMessage(mainClientDescriptor, message, messageLength);
    }
    catch (std::runtime_error& e)
    {
        std::cerr << e.what() << std::endl;
    }
}

void TCPServer::readFromClient(char* readBuffer, int32_t n)
{
    readMessage(mainClientDescriptor, readBuffer, n);
}

bool TCPServer::isMessageReady(MessageType messageType) 
{
    // Bounds checking
    if (messageType < 0 || messageType >= NUMBER_OF_MESSAGE_TYPES) {
            throw new std::runtime_error("TCPServer: invalid messagetype requested");
    }
    return messageReadyFlags[messageType];
}

void TCPServer::setMessageReady(MessageType messageType, bool ready) 
{
    // Bounds checking
    if (messageType < 0 || messageType >= NUMBER_OF_MESSAGE_TYPES) {
            throw new std::runtime_error("TCPServer: invalid messagetype requested");
    } 
    messageReadyFlags[messageType] = ready;
}

const void* TCPServer::getMessage(MessageType messageType) 
{ 
    switch (messageType) {
        case REMOTE:
            return this->remoteMessageBuffer;
        default:
            throw new std::runtime_error("TCPServer.getMessage(): requested messageType is invalid");
    }
    return nullptr;
}

void TCPServer::updateInput()
{
    int n = getMessageType();
    // n < 0 if message unavailable, so skip this logic if that's the case
    if (n >= 0) {
        MessageType messageType = static_cast<MessageType>(n);
        // For now read one message at a time otherwise concerns
        // of blocking are real as we just infinitely read. To accomplish
        // this better multithreading of some type would be important.
        switch (messageType)
        {
            case MessageType::REMOTE:
                readRemoteMessage();
                break;
            default:
                break;
        }
    }
}

void TCPServer::readRemoteMessage()
{
    readMessage(
        mainClientDescriptor,
        reinterpret_cast<char*>(remoteMessageBuffer),
        aruwlib::Remote::REMOTE_BUF_LEN);
    messageReadyFlags[REMOTE] = true;
}

int8_t TCPServer::getMessageType()
{
    int8_t messageType = -1;
    int n = read(mainClientDescriptor, &messageType, 1);
    while (n < 0)
    {
        // Socket should be non-blocking, so EAGAIN or EWOULDBLOCK means
        // that no data is available.
        if (errno & (EAGAIN | EWOULDBLOCK))
        {
            break;  // break and skip to return statement
        }
        else if (errno != EINTR)
        {
            // All other errors besides EINTR are fatal pretty much.
            perror("TCPServer: Read error");
            throw new std::runtime_error("ReadError");
        }
        // The one error that we do want to retry the read after is EINTR.
        n = read(mainClientDescriptor, &messageType, 1);
    }
    return messageType;
}

void readMessage(int16_t fileDescriptor, char* readBuffer, uint16_t messageLength)
{
    readBuffer[messageLength] = '\0';  // Null terminate the message
    uint16_t bytesRead = 0;
    while (bytesRead < messageLength)
    {
        int32_t n = read(fileDescriptor, readBuffer + bytesRead, messageLength - bytesRead);
        if (n < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            {
                continue;
            }
            else
            {
                perror("TCPServer failed to read from client");
                throw std::runtime_error("ReadingError");
            }
        }
        bytesRead += n;
    }
}

void writeMessage(int16_t fileDescriptor, const char* message, uint16_t bytes)
{
    uint32_t bytesWritten = 0;
    while (bytesWritten < bytes)
    {
        int32_t n = write(fileDescriptor, message + bytesWritten, bytes - bytesWritten);
        if (n < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            {
                continue;
            }
            else
            {
                perror("TCPServer failed to write");
                throw std::runtime_error("WriteError");
            }
        }
        bytesWritten += n;
    }
}

int32_t readInt32(int16_t fileDescriptor)
{
    char buffer[4];
    readMessage(fileDescriptor, buffer, 4);
    int32_t answer = 0;
    for (size_t i = 0; i < 4; i++)
    {
        answer = answer | buffer[i];
        answer <<= 8;
    }
    return answer;
}

// Only construct static singleton in actual sim, not in unit tests.
#ifndef ENV_UNIT_TESTS
// Definition of static variable. mainServer never created otherwise.
TCPServer TCPServer::mainServer(2001);
#endif

}  // namespace communication

}  // namespace aruwlib

#endif  // PLATFORM_HOSTED
