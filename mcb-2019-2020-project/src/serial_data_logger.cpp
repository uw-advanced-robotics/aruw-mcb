#include "src/serial_data_logger.hpp"

namespace src
{
namespace logger
{

void logger::SerialDataLogger::runLogger() {
    if (inputReceieved()) {
        writeToTerminal();
        terminalInput.clear();
    }
}

int logger::SerialDataLogger::matchToCommand() {
    // Search through array of commands
    if (!terminalInput.compare(std::string("COMMAND0")) || terminalInput.length() == 1)
    {
        terminalInput.clear();
        return 0;
    }
    else if (!terminalInput.compare(std::string("COMMAND1")))
    {
        terminalInput.clear();
        return 1;
    } else if (!terminalInput.compare(std::string("COMMAND2")))
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
}

// If after comparison with input to existing commands is true
// Switch through which one it matched with and write in response
void logger::SerialDataLogger::writeToTerminal() {
    switch(matchToCommand()) {
        case 0: loggerDevice.write("\r Case 0 Corresponding to COMMAND0\n\r");
            break;
        case 1: loggerDevice.write("\r Case 1 Corresponding to COMMAND1\n\r");
            break;
        case 2: loggerDevice.write("\r Case 2 Corresponding to COMMAND2\n\r");
            break;
        default:
            loggerDevice.write("\r invalid\n\r");
    }
}

void logger::SerialDataLogger::write(char message) {
    loggerDevice.write(message);
}

bool logger::SerialDataLogger::compare(std::string command) {
    if (!terminalInput.compare(command) || terminalInput.length() == 1) {
        terminalInput.clear();
        return true;
    }
}

} // namespace src

} // namespace logger