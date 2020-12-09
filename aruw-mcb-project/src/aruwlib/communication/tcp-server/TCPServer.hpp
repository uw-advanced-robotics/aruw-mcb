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

#ifndef TCPSERVER_HPP_
#define TCPSERVER_HPP_

#include <netinet/in.h>

#include <atomic>
#include <cstdint>

namespace aruwlib
{
namespace communication
{
/**
 * TCPServer is an singleton class for running a TCPServer using a user
 * defined messaging protocol.
 */
class TCPServer
{
public:
    /* PortNumber which the server will try to open on. This seems finicky
     * as it's possible that port is in use, but I don't know how to do 
     * better (Tenzin)*/
    static const int16_t PORT_NUMBER = 8888;
    static const uint8_t LISTEN_QUEUE_SIZE = 5; // 5 is max on most systems

    /**
     * Accepts a new connection and stores the file descriptor in mainClientDescriptor
     */
    void getConnection();

    /**
     * Post: Returns the port number of this server.
     */
    uint16_t getPortNumber();

    /**
     * Writes the null-terminated message "message" to the connected TCP client
     * if it is connected.
     */
    static void writeToClient(const char* message, int32_t messageLength);

private:
    bool socketOpened;
    bool clientConnected;
    static int16_t listenFileDescriptor; // File descriptor which server gets connection requests
    static int16_t mainClientDescriptor; // File Descriptor which we communciate with
    const static uint16_t serverPortNumber = PORT_NUMBER;
    sockaddr_in serverAddress;
    
    /**
     * Pre: 
     * Post: Creates a new TCPServer instance object, with its own unique client file
     * descriptor field and its own buffer.
     */
    TCPServer();

    /**
     * Destructor. Only special thing is that it closes any open file descriptors.
     */
    ~TCPServer();

    /**
     * Listen Loop that will be run to listen for data from the client connection.
     */
    void listenLoop();

    // Singleton server.
    static TCPServer mainServer;
};

/**
 * Read from the given fileDescriptor, ensuring to read "messageLength" bytes. readBuffer
 * must be at least of size messageLength + 1 to allow null-terminated message to fit.
 */
static void readMessage(int16_t fileDescriptor, char* readBuffer, uint16_t messageLength);

/**
 * Pre: fileDescriptor should be open.
 * Write to connected ClientFileDescriptor, ensures that all bytes are sent.
 * Throws std::runtime_error if write() fails, check errno to see exact code why.
 */
static void writeMessage(int16_t fileDescriptor, const char* message, uint16_t bytes);

/**
 * Read the next four bytes from the TCP stream as an int32_t. It is expected
 * that the lowest register bytes are most significant (big endian)
 */
static int32_t readInt32(int16_t fileDescriptor);

}  // namespace communication

}  // namespace aruwlib

#endif  // TCP_SERVER_HPP_

#endif  // PLATFORM_HOSTED
