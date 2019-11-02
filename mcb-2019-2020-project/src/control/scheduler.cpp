#include "src/control/scheduler.hpp"

namespace aruwlib
{

namespace control
{
    modm::LinkedList<Command*> Scheduler::commandList;

    modm::LinkedList<Subsystem*> Scheduler::subsystemList;

    void Scheduler::addCommand(Command* control)
    {
        commandList.append(control);
    }

    void Scheduler::run()
    {
        // loop through commands.
        for (int i = commandList.getSize(); i > 0; i--)
        {
            Command* currCommand = commandList.getFront();
            commandList.removeFront();
            currCommand->execute();
            // only add back to list if the command is not finished
            if (!currCommand->isFinished())
            {
                commandList.append(currCommand);
            }
        }

        // refresh all subsystems (i.e. run control loops where necessary)
        for (int i = subsystemList.getSize(); i > 0; i--)
        {
            Subsystem* currSubsystem = subsystemList.getFront();
            subsystemList.removeFront();
            subsystemList.append(currSubsystem);
            currSubsystem->refresh();
        }
    }

    void Scheduler::resetAll(void)
    {
        for (int i = commandList.getSize(); i > 0; i--)
        {
            Command* currCommand = commandList.getFront();
            commandList.removeFront();
            commandList.append(currCommand);
        }
    }

    void Scheduler::removeCommand(const Command* command)
    {
        for (int i = commandList.getSize(); i > 0; i--)
        {
            Command* currCommand = commandList.getFront();
            commandList.removeFront();
            if (currCommand != command)
            {
                commandList.append(currCommand);
            }
        }
    }

    bool Scheduler::isScheduled(const Command* command)
    {
        bool scheduled = false;
        for (int i = commandList.getSize(); i > 0; i--)
        {
            Command* currCommand = commandList.getFront();
            if (command == currCommand)
            {
                scheduled = true;
            }
            commandList.removeFront();
            commandList.append(currCommand);
        }
        return scheduled;
    }

    void Scheduler::addSubsystem(Subsystem* subsystem)
    {
        subsystemList.append(subsystem);
    }
}  // namespace control

}  // namespace aruwlib