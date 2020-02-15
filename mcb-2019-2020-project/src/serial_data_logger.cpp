#include "src/serial_data_logger.hpp"

namespace src
{
namespace logger
{

void logger::SerialDataLogger::runLogger() {
    if (inputReceieved()) {
        // loggerDevice.write(terminalInput.c_str());
        // parseInput();
        // terminalInput.append(" : This is terminalInput");
        // const char *c = terminalInput.c_str();
        // loggerDevice.write(c);
        // terminalInput.clear();
        writeToTerminal();
        terminalInput.clear();
    }
}

void logger::SerialDataLogger::parseInput() {
    queue.push(a);
    
    while(queue.isNotEmpty()) {
        char temp = queue.get();
        // This comparison method is worthless
        // Too many errors/mismatched datatypes 
        // Look into a different way to go about this
        terminalInput.push_back(temp);
        // loggerDevice.write(temp);
        queue.pop();
    }
   // terminalInput.append(" terminalInput");
} 

int logger::SerialDataLogger::matchToCommand() {
    // Search through array of commands
    if (!terminalInput.compare(std::string("a")) || terminalInput.length() == 1)
    {
        terminalInput.clear();
        return 1;
    }
    else if (!terminalInput.compare(std::string("bb")))
    {
        terminalInput.clear();
        return 0;
    } else if (!terminalInput.compare(std::string("COMMAND1")))
    {
        return 2;
    }
    terminalInput.clear();
    return -1;
}

// Makes sure we type something first and press enter key
bool logger::SerialDataLogger::inputReceieved() {
    // every time you read from serial
    if (loggerDevice.read(a)) {
        if (a == '\r') {
            return true;
        } else {
            terminalInput.push_back(a);
        }
    }
    return false;

    // if you read enter serial, return true
    // while (loggerDevice.read(a) && a != '\n') {  // remove while loop 
    //     return false;
    // }
    // // while (!loggerDevice.read(a)) {
    // //    a = 'b';
    // // }
    // return true;
}

// If after comparison with input to existing commands is true
// Switch through which one it matched with and write in response
void logger::SerialDataLogger::writeToTerminal() {
    switch(matchToCommand()) {
        case 0: loggerDevice.write("Case 0 Corresponding to COMMAND1\r");
            break;
        case 1: loggerDevice.write("Case 1 Corresponding to COMMAND2\r");
            break;
        case 2: loggerDevice.write("case 2");
        default:
            loggerDevice.write("incorrect");
    }
    // Reset the terminalInput buffer
    // terminalInput = "";
}

} // namespace src

} // namespace logger