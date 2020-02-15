#ifndef SERIAL_DATA_LOGGER_HPP
#define SERIAL_DATA_LOGGER_HPP

#include <modm/debug/logger.hpp>
#include <modm/container/queue.hpp>
#include <modm/container/linked_list.hpp>

#include <iostream>
#include <rm-dev-board-a/board.hpp>

using namespace std;

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
    // Read the input
    void parseInput();
    
    void writeToTerminal();

    bool inputReceieved();

    // turn data into writeable information
    // const char toString(int data);
    
    int matchToCommand();
    
    modm::Queue<char, modm::LinkedList<char>> queue;
    modm::IODeviceWrapper< Usart2, modm::IOBuffer::BlockIfFull > loggerDevice;

    char a;
    // string commands[2] = {"COMMAND1", "COMMAND2"};

    string terminalInput;

    const char *commands[2] = {"COMMAND1", "COMMAND2"};
};

} // namespace logger

} // namespace src

#endif