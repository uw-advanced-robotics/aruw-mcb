#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

// TCP Server class to allow MCB simulator to communicate with stuff.
// I actually have no idea what's happening ~ Tenzin
class TCPServer 
{
private:
    static const int BUFFER_SIZE = 1024;
    bool socketOpened = false;
    int serverPortNumber;
    int listenFileDescriptor;
    int connectedSocketFileDescriptors[10];
    sockaddr_in serverAddress;
    char buffer[BUFFER_SIZE]; // Buffer to store whatever we read from socket or for writing to a socket.
public:

    TCPServer(int portNumber) {
        memset(connectedSocketFileDescriptors, -1, sizeof(connectedSocketFileDescriptors));

        // Do sockety stuff.
        serverPortNumber = portNumber;
        listenFileDescriptor = socket(AF_INET, SOCK_STREAM, 0);
        if (listenFileDescriptor < 0) {
            std::cerr << "ERROR opening socket";
            // Put better error handling here.
            exit(1);
        }
        socketOpened = true;
        memset((char *) &serverAddress, 0, sizeof(serverAddress)); // Initialize
        // bytes of serverAddress to 0.
        serverAddress.sin_family = AF_INET;
        serverAddress.sin_port = htons(serverPortNumber);
        serverAddress.sin_addr.s_addr = INADDR_ANY;
        if (bind(listenFileDescriptor, (struct sockaddr *) &serverAddress, sizeof(serverAddress)) < 0) {
            std::cerr << "ERROR binding socket";
            // Put better error handling here.
            exit(1);
        }
        listen(listenFileDescriptor, 5);
    }

    ~TCPServer() {
        if (socketOpened) {
            close(listenFileDescriptor);
        }
    }
    
    // I assume I'll need to learn some threading for this??
    // Accept a new client socket connection.
    void connectSocket() {
        sockaddr_in clientAddress;
        socklen_t clientAddressLength = sizeof(clientAddress);
        connectedSocketFileDescriptors[0] = accept(listenFileDescriptor, (sockaddr *) &clientAddress,
                                                   &clientAddressLength);
        if (connectedSocketFileDescriptors[0] < 0) {
            std::cerr << "ERROR on accept";
            exit(1);
        }
    }

    // post: Sets buffer equal to as many bytes as can be read from the given file descriptor
    // Returns pointer to its buffer, nullptr if operation failed.
    char* readFromSocket(int fileDescriptor, int bytesToRead) {
        // 0 initialize entire buffer.
        memset(buffer, 0, sizeof(buffer));
        int totalBytesRead = 0;
        while (totalBytesRead < bytesToRead) {
            int bytesRead = read(fileDescriptor, buffer, std::min(bytesToRead - totalBytesRead, BUFFER_SIZE - 1));
            if (bytesRead < 0) {
                std::cerr << "Reading from socket failed";
                return nullptr;
            }
            totalBytesRead += bytesRead;
        }
        // UNFINISHED
        return buffer;
    }

    // Write to a fileDescriptor, returns whether or not writing operation was successful
    bool writeToSocket(int fileDescriptor, char* messagePointer, int bytes) {
        int n = write(fileDescriptor, messagePointer, bytes);
        if (n < 0) {
            std::cerr << "Writing to socket failed\n";
            return false;
        }
        return true;
    }

    virtual void respondToPrompt();
};