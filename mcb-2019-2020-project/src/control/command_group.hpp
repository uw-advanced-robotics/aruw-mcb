#ifndef __COMMAND_GROUP_HPP__
#define __COMMAND_GROUP_HPP__

#include <modm/container/dynamic_array.hpp>
#include "command.hpp"

namespace aruwlib
{

namespace control
{

/**
 * A CommandGroup is a list of commands that constitute a more complex
 * command and can be implemented how the user sees fit (i.e. using state
 * machine, etc)
 *
 * @see Command
 * @see Subsystem
 */
class CommandGroup : public Command
{
    CommandGroup() : commands(5), subsystemRequirements(5)
    {}

    ~CommandGroup() = default;

    int getCommandSize() const
    {
        return commands.getSize();
    }

    virtual void Initialize(void) = 0;

    virtual void execute(void) = 0;

    virtual bool isFinished(void) = 0;

    virtual void end(bool interrupted) = 0;

    /*
     * Typically call in constructor of command group implementation.
     * For all commands in command group, add them to the list of
     * required commands so an appropriate list of subsystem
     * requirements can be formulated.
     */
    void addCommandRequirement(modm::SmartPointer command);

 private:
    int currentCommandIndex = -1;

    modm::DynamicArray<modm::SmartPointer> commands;

    modm::DynamicArray<modm::SmartPointer> subsystemRequirements;
};

}  // namespace control

}  // namespace aruwlib

#endif
