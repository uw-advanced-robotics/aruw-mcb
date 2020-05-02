#ifndef SERIAL_HANDLER_HPP
#define SERIAL_HANDLER_HPP

#include "serial_data_logger.hpp"

namespace src
{
namespace logger
{

// This should handle switching between groups/types of input? 
// Will do the heavy parsing of the user input
// If the user input matches an existing command go to that subgroup
// Execution example:

// "e show_errors"
// -> depending on prefix switch group
// if the prefix doesn't exist print an error
// inside switchGroup it calls the general run method of that specific subgroup
// the subgroup then runs through the rest of the remaining string to do what it wants
// if the command doesn't exist, print and error
// otherwise do what it wants
class SerialHandler
{
    public:

    void switchGroup();

    private:

    std::string userInput;
};


} // namespace logger

} // namespace src

#endif
