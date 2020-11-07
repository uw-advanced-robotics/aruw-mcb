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

#ifndef ARUWMCBPROJECT_ARUWLIB_COMMUNICATION_TCPSERVER_HPP_
#define ARUWMCBPROJECT_ARUWLIB_COMMUNICATION_TCPSERVER_HPP_

#include <netinet/in.h>
#include <cstdint>

namespace aruwlib
{
namespace communication
{
class TCPServer
{
public:
    static const uint32_t BUFFER_SIZE = 257; // One larger than message length
        // to store a null character at the end to terminate string.
    static const uint32_t MESSAGE_LENGTH =
        256;  // Number of bytes in the message stored in last read message.

    TCPServer(uint16_t portNumber);

    ~TCPServer();

    // Post: Accept a new client connection. Closes old connection if there was one and
    // then sets "clientFileDescriptor" to be new connection's descriptor.
    void acceptConnection();

    // Post: Reads a message to the class's buffer ensuring that MESSAGE_LENGTH bytes are
    // read, before finally returning a pointer to the beginning of the buffer.
    const unsigned char* readMessage();

    // Write to connected ClientFileDescriptor, ensures that all bytes are sent.
    void writeToClient(unsigned char* message, uint16_t bytes);

    // Post: Returns the port number of this server.
    uint16_t getPortNumber();

private:
    bool socketOpened;
    bool clientConnected;
    uint16_t serverPortNumber;
    int16_t listenFileDescriptor;
    int16_t clientFileDescriptor;
    sockaddr_in serverAddress;
    unsigned char buffer[BUFFER_SIZE];
};

}  // namespace communication

}  // namespace aruwlib

#endif // ARUWMCBPROJECT_ARUWLIB_COMMUNICATION_TCPSERVER_HPP_

#endif // PLATFORM_HOSTED
