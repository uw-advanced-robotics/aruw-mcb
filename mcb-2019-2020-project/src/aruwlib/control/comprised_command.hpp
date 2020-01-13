#ifndef __COMPRISED_COMMAND_HPP__
#define __COMPRISED_COMMAND_HPP__

#include "command.hpp"
#include <modm/container/dynamic_array.hpp>
#include <modm/container/smart_pointer.hpp>

namespace aruwlib
{

namespace control
{

class ComprisedCommand : public Command
{
 public:
    bool usesCommand(modm::SmartPointer& command);

    void addUseCommand(modm::SmartPointer& commandToAdd);

 private:
    modm::DynamicArray<modm::SmartPointer> commandsToUse;
};

}  // namespace control

}  // namespace aruwlib

#endif
