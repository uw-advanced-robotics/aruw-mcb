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

#ifndef TAPROOT_TCP_SERVER_HPP_
#define TAPROOT_TCP_SERVER_HPP_

#ifdef PLATFORM_HOSTED

#ifdef __linux__
#include <netinet/in.h>
#endif

#include <atomic>
#include <cstdint>

namespace tap
{
namespace communication
{
/**
 * TCPServer is an singleton class for running a TCPServer using a user
 * defined messaging protocol.
 */
class TCPServer
{
// Make constructor and destructor public if in unit tests environment.
#ifdef ENV_UNIT_TESTS
public:
#endif
    /**
     * Post: Creates a new TCPServer and binds to the first avaiable port
     * in the range [portnumber, portnumber+1000]. If it cannot succesfully
     * bind to any of these ports, throws a std::runtime_error.
     */
    TCPServer(int portnumber);

    /**
     * Destructor. Only special thing is that it closes any open file descriptors.
     */
    ~TCPServer();

public:
    /* PortNumber which the server will try to open on. This seems finicky
     * as it's possible that port is in use, but I don't know how to do
     * better (Tenzin)*/
    static const int16_t PORT_NUMBER = 8888;
    static const uint8_t LISTEN_QUEUE_SIZE = 5;  // 5 is max on most systems

    /**
     * Return a const reference to the singleton static instance of this class.
     */
    static TCPServer* MainServer();

    /**
     * Accepts a new connection and stores the file descriptor in mainClientDescriptor
     */
    void getConnection();

    /**
     * Closes mainClientFile descriptor and then sets its value to -1.
     */
    void closeConnection();

    /**
     * Post: Returns the port number of this server.
     */
    uint16_t getPortNumber();

    /**
     * Writes the null-terminated message "message" to the connected TCP client
     * if there is one.
     */
    void writeToClient(const char* message, int32_t messageLength);

private:
#ifdef __linux__
    bool socketOpened;
    bool clientConnected;
    int16_t listenFileDescriptor;  // File descriptor which server gets connection requests
    int16_t mainClientDescriptor;  // File Descriptor which we communciate with
    sockaddr_in serverAddress;
    int16_t portNumber;  // portNumber the server is bound to
#endif                   // __linux__

    // Singleton server.
    static TCPServer mainServer;
};  // TCPServer

#ifdef __linux__
/**
 * Pre: Message readBuffer must be messageLength + 1 bytes long otherwise out of bounds
 * array access will occur and behavior will be undefined.
 * Post: Reads a message to the given "readBuffer" ensuring that messageLength bytes are
 * read.
 *
 * Throws: runtime_error if read() fails.
 */
void readMessage(int16_t fileDescriptor, char* readBuffer, uint16_t messageLength);

/**
 * Pre: fileDescriptor should be open.
 * Write to connected ClientFileDescriptor, ensures that all bytes are sent.
 * Throws std::runtime_error if write() fails, check errno to see exact code why.
 */
void writeMessage(int16_t fileDescriptor, const char* message, uint16_t bytes);

/**
 * Read the next four bytes from the TCP stream as an int32_t. It is expected
 * that the lowest register bytes are most significant (big endian)
 */
int32_t readInt32(int16_t fileDescriptor);
#endif  // __linux__

}  // namespace communication

}  // namespace tap

#endif  // PLATFORM_HOSTED

#endif  // TAPROOT_TCP_SERVER_HPP_
