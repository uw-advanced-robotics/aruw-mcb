#include "src/control/scheduler.hpp"

namespace aruwlib
{

namespace control
{
    modm::LinkedList<Command*> Scheduler::commandList;

    void Scheduler::addCommand(Command* control)
    {
        commandList.append(control);
    }

    void Scheduler::run()
    {
        // loop through commands.
        for (int i = commandList.getSize(); i >= 0; i--)
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
    }

    void Scheduler::resetAll(void)
    {
        for (int i = commandList.getSize(); i >= 0; i--)
        {
            Command* currCommand = commandList.getFront();
            commandList.removeFront();
            commandList.append(currCommand);
        }
    }

    static void removeCommand(Command* command)
    {
        // TODO(matthew)
    }

    bool Scheduler::isScheduled(const Command* command)
    {
        bool scheduled = false;
        for (int i = commandList.getSize(); i >= 0; i--)
        {
            Command* currCommand = commandList.getFront();
            if (command = currCommand)
            {
                scheduled = true;
            }
            commandList.removeFront();
            commandList.append(currCommand);
        }
    }

}  // namespace control

}  // namespace aruwlib