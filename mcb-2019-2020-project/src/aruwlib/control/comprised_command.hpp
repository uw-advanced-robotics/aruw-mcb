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
    ~ComprisedCommand();

    bool usesCommand(modm::SmartPointer commandToFind);

    void addUseCommand(const modm::SmartPointer& commandToAdd);

 private:
    modm::DynamicArray<modm::SmartPointer> commandsToUse;
};

}  // namespace control

}  // namespace aruwlib

#endif
