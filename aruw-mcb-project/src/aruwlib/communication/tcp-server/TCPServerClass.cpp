#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <exception>
#include <iostream>
// No idea if my include order is right, and honestly, too hard to check.
#include "TCPServerClass.hpp"
using std::cerr;
// TCP Server class to allow MCB simulator to communicate with stuff.
// I actually have no idea what's happening ~ Tenzin
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
        // Put better error handling here.
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
        // Put better error handling here.
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

// I assume I'll need to learn some threading for this??
// Accept a new client socket connection. Store file des
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

// Post: Reads a message to the class's buffer ensuring that MESSAGE_LENGTH bytes are
// read, before finally returning a pointer to the beginning of the buffer.
unsigned char *TCPServer::readMessage()
{
    if (!clientConnected)
    {
        cerr << "Not connected to a client";
        return nullptr;
    }
    else
    {
        memset(buffer, 0, sizeof(buffer));
        uint16_t n = read(clientFileDescriptor, buffer, MESSAGE_LENGTH);
        if (n < MESSAGE_LENGTH)
        {
            uint16_t bytesRead = n;
            while (bytesRead < MESSAGE_LENGTH)
            {
                bytesRead +=
                    read(clientFileDescriptor, buffer + bytesRead, MESSAGE_LENGTH - bytesRead);
            }
        }
        return buffer;
    }
}

// Write to connected ClientFileDescriptor, ensures that all bytes are sent.
void TCPServer::writeToClient(char *message, uint16_t bytes)
{
    uint16_t bytesWritten = write(clientFileDescriptor, message, bytes);
    while (bytesWritten < bytes)
    {
        bytesWritten += write(clientFileDescriptor, message + bytesWritten, bytes - bytesWritten);
    }
}

// Post: Returns the port number of this server.
uint16_t TCPServer::getPortNumber() { return this->serverPortNumber; }

}  // namespace communication

}  // namespace aruwlib