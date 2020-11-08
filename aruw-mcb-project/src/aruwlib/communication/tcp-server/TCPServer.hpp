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

#include <cstdint>

namespace aruwlib
{
namespace communication
{
/**
 * TCPServer provides an interface for using the POSIX sockets API. It tries it's
 * best to abstract away the funkier aspects of the sockets API.
 */
class TCPServer
{
public:
    static const uint32_t MESSAGE_LENGTH = 256;
    static const uint8_t LISTEN_QUEUE_SIZE = 5;

    /**
     * Pre: Portnumber must not be in use on current machine, throws a string exception
     * otherwise
     *
     * Post: Creates a new TCPServer instance object, with its own unique client file
     * descriptor field and its own buffer.
     */
    TCPServer(uint16_t portNumber);

    /**
     * Desctructor. Only special thing is that it closes any open file descriptors.
     */
    ~TCPServer();

    /**
     * Post: Accept a new client connection. Closes old connection if there was one and
     * then sets "clientFileDescriptor" to be new connection's descriptor.
     */
    void acceptConnection();

    /**
     * Pre: messageLength <= space allocated for "readBuffer"
     * 
     * Post: Reads a message to the given "readBuffer" ensuring that messageLength bytes are
     * read.
     * 
     * Throws: runtime_error if read() fails.
     */
    void readMessage(unsigned char* readBuffer, uint16_t messageLength);

    /**
     * Write to connected ClientFileDescriptor, ensures that all bytes are sent.
     */
    void writeToClient(const unsigned char* message, uint16_t bytes);

    /**
     * Post: Returns the port number of this server.
     */
    uint16_t getPortNumber();

private:
    bool socketOpened;
    bool clientConnected;
    int16_t listenFileDescriptor;
    int16_t clientFileDescriptor;
    uint16_t serverPortNumber;
    sockaddr_in serverAddress;

    /**
     * Read the next four bytes from the TCP stream as an int32_t. It is expected
     * that the lowest register bytes are most significant (big endian)
     */
    int32_t readInt32();
};

}  // namespace communication

}  // namespace aruwlib

#endif  // ARUWMCBPROJECT_ARUWLIB_COMMUNICATION_TCPSERVER_HPP_

#endif  // PLATFORM_HOSTED
