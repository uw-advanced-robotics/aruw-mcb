#include "src/control/scheduler.hpp"
#include "src/motor/dji_motor_tx_handler.hpp"
#include "src/communication/can/can_rx_handler.hpp"

namespace aruwlib
{

namespace control
{
    modm::LinkedList<Command*> Scheduler::commandList;

    modm::LinkedList<Subsystem*> Scheduler::subsystemList;

    bool Scheduler::addCommand(Command* control)
    {
        // only add the command if (a) command is not already being run and (b) all
        // subsystem dependencies can be interrupted.
        if (isScheduled(control))
        {
            return false;
        }

        // Check to make sure the control you are trying to add to the scheduler
        // can be added.
        // If there are command dependencies that can't be interrupted, don't schedule.
        for (const Subsystem* dependentSubsystem : control->getRequirements())
        {
            if (
                dependentSubsystem->GetCurrentCommand() != nullptr
                && !dependentSubsystem->GetCurrentCommand()->isInterruptiable()
            ) {
                return false;
            }
        }

        // // If we can replace the command based of the command dependencies, do so.
        // // O(n^2) :`( At least it looks cleaner now, (:
        // this is an iterator to a pointer to a Subsystem pointer. Whoo hoo!
        modm::LinkedList<Subsystem*>::iterator subsystemListItr = subsystemList.begin();
        while ((*subsystemListItr) != nullptr)
        {
            Subsystem* currSubsystem = (*subsystemListItr);
            ++subsystemListItr;
            // set the command to the desired command for every subsystem dependency
            for (const Subsystem* dependentSubsystem : control->getRequirements())
            {
                if (dependentSubsystem == currSubsystem)
                {
                    if (currSubsystem->GetCurrentCommand() != nullptr)
                    {
                        // end and indicate the command was interrupted
                        currSubsystem->GetCurrentCommand()->end(true);
                    }
                    control->initialize();
                    currSubsystem->SetCurrentCommand(control);
                }
            }
        }
        commandList.append(control);
        control->initialize();
        return true;
    }

    void Scheduler::run()
    {
        // refresh all subsystems (i.e. run control loops where necessary)
        // additionally, check if no command is running, in which case run the
        // default command.
        modm::LinkedList<Subsystem*>::iterator subsystemListItr = subsystemList.begin();
        while ((*subsystemListItr) != nullptr)
        {
            Subsystem* currSubsystem = *(subsystemListItr);
            ++subsystemListItr;
            // schedule default command if necessary
            if (
                currSubsystem->GetCurrentCommand() == nullptr
                && currSubsystem->GetDefaultCommand() != nullptr)
            {
                currSubsystem->GetDefaultCommand()->schedule();
            }
            currSubsystem->refresh();
        }

        // // loop through commands.
        modm::LinkedList<Command*>::iterator commandListItr = commandList.begin();
        while (*(commandListItr) != nullptr)
        {
            Command* currCommand = *(commandListItr);
            currCommand->execute();
            // only add back to list if the command is not finished
            if (!currCommand->isFinished())
            {
                ++currCommand;
                commandList.append(currCommand);
            }
            else  // end and remove command from the commandList
            {
                currCommand->end(false);
                commandListItr = commandList.remove(commandListItr);
            }
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
        modm::LinkedList<Command*>::const_iterator commandListItr = commandList.begin();
        while ((*commandListItr) != nullptr)
        {
            Command* currCommand = commandList.getFront();
            ++commandListItr;
            if (command == currCommand)
            {
                return true;
            }
        }
        return false;
    }

    bool Scheduler::registerSubsystem(Subsystem* subsystem)
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
