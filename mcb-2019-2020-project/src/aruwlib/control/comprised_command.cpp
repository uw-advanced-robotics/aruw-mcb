#include "comprised_command.hpp"

namespace aruwlib
{

namespace control
{

bool ComprisedCommand::usesCommand(modm::SmartPointer commandToFind)
{
    return commandsToUse.find(commandToFind) != commandsToUse.end();
}

void ComprisedCommand::addUseCommand(const modm::SmartPointer& commandToAdd)
{
    this->commandsToUse.append(commandToAdd);
}

}  // namespace control

}  // namespace aruwlib
