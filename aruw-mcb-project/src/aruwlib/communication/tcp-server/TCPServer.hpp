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
 * TCPServer is an abstract base class for running a TCPServer using a user
 * defined messaging protocol. The juice of this class is in its mainLoop() method
 * which is run in a new thread for each client that connects to this server. Every
 * derived class of TCPServer needs to implement and override the mainLoop() method.
 */
class TCPServer
{
public:
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
     * Destructor. Only special thing is that it closes any open file descriptors.
     */
    ~TCPServer();

    /**
     * Start the servers main listening thread. Creates a thread for listening for new
     * connections, and begins accepting connections and generating new threads once accepted.
     */
    void start();

    /**
     * Stop the servers main listening thread.
     */
    void stop();

    /**
     * Post: Returns the port number of this server.
     */
    uint16_t getPortNumber();

private:
    bool socketOpened;
    bool clientConnected;
    int16_t listenFileDescriptor;
    uint16_t serverPortNumber;
    sockaddr_in serverAddress;

    std::atomic<bool> running_;

    /**
     * run() runs a constant listen loop for new connections and creates new
     * handler threads for each accepted connection.
     */
    void run();

    void clientLoop(int16_t);
    /**
     * MainLoop that will be run for each accepted connection. Should take
     * an int16_t representing the file descriptor of the connection.
     */
    virtual void mainLoop(int16_t) = 0;

protected:
    /**
     * Read from the given fileDescriptor, ensuring to read "messageLength" bytes.
     */
    static void readMessage(int16_t fileDescriptor, char* readBuffer, uint16_t messageLength);

    /**
     * Write to connected ClientFileDescriptor, ensures that all bytes are sent.
     */
    static void writeMessage(int16_t fileDescriptor, const char* message, uint16_t bytes);

    /**
     * Read the next four bytes from the TCP stream as an int32_t. It is expected
     * that the lowest register bytes are most significant (big endian)
     */
    static int32_t readInt32(int16_t fileDescriptor);

};  // namespace communication

}  // namespace communication

}  // namespace aruwlib

#endif  // TCP_SERVER_HPP_

#endif  // PLATFORM_HOSTED
