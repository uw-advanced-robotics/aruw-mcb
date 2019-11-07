#include "src/control/scheduler.hpp"

namespace aruwlib
{

namespace control
{
    modm::LinkedList<Command*> Scheduler::commandList;

    modm::LinkedList<Subsystem*> Scheduler::subsystemList;

    void Scheduler::addCommand(Command* control)
    {
        modm::LinkedList<const Subsystem*>& commandDependencies = control->getRequirements();

        // Check to make sure the control you are trying to add to the scheduler can be added.
        bool dependencyInUse = false;

        for (int i = subsystemList.getSize(); i > 0; i--)
        {
            for (int j = commandDependencies.getSize(); j > 0; j--)
            {
                const Subsystem* subsystemDependency = commandDependencies.getFront();
                commandDependencies.removeFront();
                commandDependencies.append(subsystemDependency);
                if (
                    subsystemDependency->GetCurrentCommand() != nullptr
                    && !subsystemDependency->GetCurrentCommand()->isInterruptiable()
                ) {
                    dependencyInUse = true;
                }
            }
        }

        // If we can replace the command based of the command dependencies, do so.
        if (!dependencyInUse)
        {
            // // for all subsystems that this command is depend upon, end the current command
            // // being run.
            // for (int i = commandDependencies.getSize(); i > 0; i--)
            // {
            //     // The current command has been interrupted. End it.
            //     commandDependencies.getFront()->GetCurrentCommand()->end(true);
            //     // Set current command to something new.
            //     commandDependencies.getFront()->SetCurrentCommand(control);
            // }
            // commandList.append(control);
        }
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
        // additionally, check if no command is running, in which case run the
        // default command.
        for (int i = subsystemList.getSize(); i > 0; i--)
        {
            Subsystem* currSubsystem = subsystemList.getFront();
            subsystemList.removeFront();
            subsystemList.append(currSubsystem);
            if (currSubsystem->GetCurrentCommand() == nullptr)
            {
                if (currSubsystem->GetDefaultCommand() != nullptr)
                {
                    currSubsystem->GetDefaultCommand()->schedule();
                    // execute here so no execution cycles are skipped
                    currSubsystem->GetDefaultCommand()->execute(); 
                }
            }
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

    bool Scheduler::addSubsystem(Subsystem* subsystem)
    {
        bool subsystemAlreadyAdded = false;
        for (int i = subsystemList.getSize(); i > 0; i--)
        {
            Subsystem* currSubsystem = subsystemList.getFront();
            subsystemList.removeFront();
            subsystemList.append(currSubsystem);
            if (currSubsystem == subsystem)
            {
                subsystemAlreadyAdded = true;
            }
        }
        if (!subsystemAlreadyAdded)
        {
            subsystemList.append(subsystem);
        }
        return subsystemAlreadyAdded;
    }
}  // namespace control

}  // namespace aruwlib
