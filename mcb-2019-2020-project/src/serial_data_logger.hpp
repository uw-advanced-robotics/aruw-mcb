#ifndef SERIAL_DATA_LOGGER_HPP
#define SERIAL_DATA_LOGGER_HPP

#include <modm/debug/logger.hpp>
#include <modm/container/queue.hpp>
#include <modm/container/linked_list.hpp>

#include <iostream>
#include <rm-dev-board-a/board.hpp>

namespace src
{

namespace logger
{
    enum Command {
        COMMAND1 = 0, COMMAND2
    };

class SerialDataLogger
{
    public:
    // Runs whole program
    void runLogger();

    private:
    // Writes output to PuTTY window
    void writeToTerminal();
    // Buffers input until enter key is read
    // PuTTY reads enter keys as '\r'
    bool inputReceieved();
    // Switch block that matches input with a specific output
    int matchToCommand();
    
    // Writes a given string
    void write(char message);

    // Compares an input with an existing command
    bool compare(std::string command);
    
    modm::Queue<char, modm::LinkedList<char>> queue;
    modm::IODeviceWrapper< Usart2, modm::IOBuffer::BlockIfFull > loggerDevice;
    
    char a;

    std::string terminalInput;

};

} // namespace logger

} // namespace src

#endif