#include "src/control/scheduler.hpp"
#include "src/motor/dji_motor_tx_handler.hpp"
#include "src/communication/can/can_rx_handler.hpp"

using namespace std;

namespace aruwlib
{

namespace control
{
    // modm::LinkedList<Command*> Scheduler::commandList;

    // modm::LinkedList<Subsystem*> Scheduler::subsystemList;

    set<Subsystem*> Scheduler::subsystemList1;

    set<Command*> Scheduler::commandList1; 

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

        for (const Subsystem* dependentSubsystem : control->getRequirements())
        {
            Subsystem* isDependentSubsystem = (*subsystemList1.find(
                const_cast<Subsystem*>(dependentSubsystem)));  // im sry
            if (isDependentSubsystem == nullptr)
            {
                if (dependentSubsystem->GetCurrentCommand() != nullptr)
                {
                    // end and indicate command was interrupted
                    isDependentSubsystem->GetCurrentCommand()->end(true);
                }
                control->initialize();
                control->execute();
            }
        }

        // // If we can replace the command based of the command dependencies, do so.
        // // O(n^2) :`( At least it looks cleaner now, (:
        // this is an iterator to a pointer to a Subsystem pointer. Whoo hoo!
        // modm::LinkedList<Subsystem*>::iterator subsystemListItr = subsystemList.begin();
        // while ((*subsystemListItr) != nullptr)
        // {
        //     Subsystem* currSubsystem = (*subsystemListItr);
        //     // set the command to the desired command for every subsystem dependency
        //     for (const Subsystem* dependentSubsystem : control->getRequirements())
        //     {
        //         if (dependentSubsystem == currSubsystem)
        //         {
        //             if (currSubsystem->GetCurrentCommand() != nullptr)
        //             {
        //                 // end and indicate the command was interrupted
        //                 currSubsystem->GetCurrentCommand()->end(true);
        //             }
        //             control->initialize();
        //             currSubsystem->SetCurrentCommand(control);
        //         }
        //     }
        //     ++subsystemListItr;
        // }
        // commandList.append(control);
        // control->initialize();
        // return true;
    }

    void Scheduler::run()
    {
        for (auto subsystemListItr1 = subsystemList1.begin();
            subsystemListItr1 != subsystemList1.end();
        ) {
           if (
               (*subsystemListItr1)->GetCurrentCommand() == nullptr
                && (*subsystemListItr1)->GetDefaultCommand() != nullptr
            ) {
                (*subsystemListItr1)->GetDefaultCommand()->schedule();
            }
            (*subsystemListItr1)->refresh();
        }
        // refresh all subsystems (i.e. run control loops where necessary)
        // additionally, check if no command is running, in which case run the
        // default command.
        // modm::LinkedList<Subsystem*>::iterator subsystemListItr = subsystemList.begin();
        // while ((*subsystemListItr) != nullptr)
        // {
        //     Subsystem* currSubsystem = *(subsystemListItr);
        //     // schedule default command if necessary
        //     if (
        //         currSubsystem->GetCurrentCommand() == nullptr
        //         && currSubsystem->GetDefaultCommand() != nullptr)
        //     {
        //         currSubsystem->GetDefaultCommand()->schedule();
        //     }
        //     currSubsystem->refresh();
        //     ++subsystemListItr;
        // }

        for (auto commandList1Itr = commandList1.begin();
            commandList1Itr != commandList1.end();
        ) {
            (*commandList1Itr)->execute();
            if ((*commandList1Itr)->isFinished())
            {
                commandList1Itr = commandList1.erase(commandList1Itr);
            }
            else
            {
                ++commandList1Itr;
            }
        }

        // loop through commands.
        // modm::LinkedList<Command*>::iterator commandListItr = commandList.begin();
        // while (*(commandListItr) != nullptr)
        // {
        //     Command* currCommand = *(commandListItr);
        //     currCommand->execute();
        //     // end command if finished executing
        //     if (currCommand->isFinished())
        //     {
        //         currCommand->end(false);
        //         commandListItr = commandList.remove(commandListItr);
        //     }
        //     ++commandListItr;
        // }
    }

    void Scheduler::removeCommand(Command* command)
    {
        set<Command*>::iterator commandList1Itr = commandList1.find(command);
        if (*commandList1Itr != nullptr)
        {
            (*commandList1Itr)->end(false);
            commandList1.erase(commandList1Itr);
        }

        // modm::LinkedList<Command*>::iterator commandListItr = commandList.begin();
        // while (*(commandListItr) != nullptr)
        // {
        //     Command* currCommand = *(commandListItr);
        //     if (currCommand == command)
        //     {
        //         commandListItr = commandList.remove(commandListItr);
        //     }
        //     ++commandListItr;
        // }
    }

    bool Scheduler::isScheduled(Command* command)
    {
        return *(commandList1.find(command)) != nullptr;

        // modm::LinkedList<Command*>::const_iterator commandListItr = commandList.begin();
        // while ((*commandListItr) != nullptr)
        // {
        //     Command* currCommand = commandList.getFront();
        //     if (command == currCommand)
        //     {
        //         return true;
        //     }
        //     ++commandListItr;
        // }
        // return false;
    }

    void Scheduler::registerSubsystem(Subsystem* subsystem)
    {
        auto subsystemListItr1 = subsystemList1.find(subsystem);
        if (*subsystemListItr1 != nullptr)
        {
            subsystemList1.insert(subsystem);
        }

        // bool subsystemAlreadyAdded = false;
        // modm::LinkedList<Subsystem*>::iterator subsystemListItr = subsystemList.begin();
        // while (*subsystemListItr != nullptr)
        // {
        //     Subsystem* currSubsystem = *subsystemListItr;
        //     if (currSubsystem == subsystem)
        //     {
        //         subsystemAlreadyAdded = true;
        //     }
        //     ++subsystemListItr;
        // }
        // if (!subsystemAlreadyAdded)
        // {
        //     subsystemList.append(subsystem);
        // }
    }
}  // namespace control

}  // namespace aruwlib
