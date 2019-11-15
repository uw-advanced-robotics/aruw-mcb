#include "src/control/scheduler.hpp"
#include "src/motor/dji_motor_tx_handler.hpp"
#include "src/communication/can/can_rx_handler.hpp"

using namespace std;

namespace aruwlib
{

namespace control
{
    set<Subsystem*> CommandScheduler::subsystemList;

    set<Command*> CommandScheduler::commandList;

    bool CommandScheduler::addCommand(Command* commandToAdd)
    {
        // only add the command if (a) command is not already being run and (b) all
        // subsystem dependencies can be interrupted.
        if (isCommandScheduled(commandToAdd))
        {
            return false;
        }

        // Check to make sure the commandToAdd you are trying to add to the scheduler
        // can be added.
        // If there are command dependencies that can't be interrupted, don't schedule.
        for (auto requirement : commandToAdd->getRequirements())
        {
            if (!isSubsystemRegistered(requirement)
                && requirement->GetCurrentCommand() != nullptr
                && !(requirement->GetCurrentCommand()->isInterruptible())
            ) {
                return false;
            }
        }

        // end all commands running on the subsystem requirements. 
        for (auto requirement : commandToAdd->getRequirements())
        {
            set<Subsystem*>::iterator isDependentSubsystem = subsystemList.find(
                const_cast<Subsystem*>(requirement));  // im sry
            if (isDependentSubsystem != subsystemList.end())
            {
                if (requirement->GetCurrentCommand() != nullptr)
                {
                    requirement->GetCurrentCommand()->end(true);
                }
                (*isDependentSubsystem)->SetCurrentCommand(commandToAdd);
            }
        }
        commandList.insert(commandToAdd);
        commandToAdd->initialize();
        return true;
    }

    void CommandScheduler::run()
    {
        for (auto currSubsystem : subsystemList)
        {
            if (currSubsystem->GetCurrentCommand() == nullptr
                && currSubsystem->GetDefaultCommand() != nullptr) {
                addCommand(currSubsystem->GetDefaultCommand());
            }
            currSubsystem->refresh();
        }

        for (auto commandList1Itr = commandList.begin();
            commandList1Itr != commandList.end();
        ) {
            (*commandList1Itr)->execute();
            if ((*commandList1Itr)->isFinished())
            {
                (*commandList1Itr)->end(false);
                commandList1Itr = commandList.erase(commandList1Itr);
            }
            else
            {
                ++commandList1Itr;
            }
        }
    }

    void CommandScheduler::removeCommand(Command* command)
    {
        set<Command*>::iterator commandList1Itr = commandList.find(command);
        if (commandList1Itr != commandList.end())
        {
            (*commandList1Itr)->end(false);
            commandList.erase(commandList1Itr);
        }
    }

    bool CommandScheduler::isSubsystemRegistered(const Subsystem* subsystem)
    {
        return subsystemList.find(const_cast<Subsystem*>(subsystem)) != subsystemList.end();
    }

    bool CommandScheduler::isCommandScheduled(Command* command)
    {
        return commandList.find(command) != commandList.end();
    }

    void CommandScheduler::registerSubsystem(Subsystem* subsystem)
    {
        if (!isSubsystemRegistered(subsystem))
        {
            subsystemList.insert(subsystem);
        }
    }
}  // namespace control

}  // namespace aruwlib
