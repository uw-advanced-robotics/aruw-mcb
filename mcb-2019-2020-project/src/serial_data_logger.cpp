#include "src/serial_data_logger.hpp"

namespace src
{
namespace logger
{

void logger::SerialDataLogger::runLogger() {
    if (inputReceieved() == false) {
        parseInput();
        // terminalInput.append(" : This is terminalInput");
        // const char *c = terminalInput.c_str();
        // loggerDevice.write(c);
        // terminalInput.clear();
        writeToTerminal();
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
    const char *c = terminalInput.c_str();
    loggerDevice.write(c);
    const char *v = "COMMAND1";
    int num = -1;

    if (c == v) {
        num = 0;
    }

    if (terminalInput == std::string("COMMAND2")) {
        num = 1;
    }
    
    // int index = 0;
    // for (const char *command : commands) {
    //    if (command == c) {
    //        return index;
    //    } else {
    //        index++;
    //    }   
    // }
    return num;
}

// Makes sure we type something first and press enter key
bool logger::SerialDataLogger::inputReceieved() {
    while (loggerDevice.read(a) && a != '\n') {
        return false;
    }
    // while (!loggerDevice.read(a)) {
    //    a = 'b';
    // }
    return true;
}

// If after comparison with input to existing commands is true
// Switch through which one it matched with and write in response
void logger::SerialDataLogger::writeToTerminal() {
    switch(matchToCommand()) {
        case 0: loggerDevice.write("Case 0 Corresponding to COMMAND1");
            break;
        case 1: loggerDevice.write("Case 1 Corresponding to COMMAND2");
            break;
    }
    // Reset the terminalInput buffer
    terminalInput.clear();
    // terminalInput = "";
}

} // namespace src

} // namespace logger