#ifndef ARUWMCBPROJECT_ARUWLIB_COMMUNICATION_TCPSERVER_HPP_
#define ARUWMCBPROJECT_ARUWLIB_COMMUNICATION_TCPSERVER_HPP_
#endif

#include <netinet/in.h>

#include <cstdint>

namespace aruwlib
{
namespace communication
{
class TCPServer
{
public:
    static const uint32_t BUFFER_SIZE = 65536;
    static const uint32_t MESSAGE_LENGTH =
        256;  // Number of bytes in the message stored in last read message.

    TCPServer(uint16_t portNumber);

    ~TCPServer();

    // Post: Accept a new client connection. Closes old connection if there was one and
    // then sets "clientFileDescriptor" to be new connection's descriptor.
    void acceptConnection();

    // Post: Reads a message to the class's buffer ensuring that MESSAGE_LENGTH bytes are
    // read, before finally returning a pointer to the beginning of the buffer.
    unsigned char* readMessage();

    // Write to connected ClientFileDescriptor, ensures that all bytes are sent.
    void writeToClient(char* message, uint16_t bytes);

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