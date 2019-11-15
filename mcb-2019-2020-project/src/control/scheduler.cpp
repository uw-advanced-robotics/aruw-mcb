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

    bool CommandScheduler::addCommand(Command* control)
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
        for (auto requirement : control->getRequirements())
        {
            if (requirement->GetCurrentCommand() != nullptr
                && !(requirement->GetCurrentCommand()->isInterruptible())
            ) {
                return false;
            }
        }

        // end all commands running on the subsystem requirements. 
        for (auto requirement : control->getRequirements())
        {
            set<Subsystem*>::iterator isDependentSubsystem = subsystemList.find(
                const_cast<Subsystem*>(requirement));  // im sry
            if (isDependentSubsystem != subsystemList.end())
            {
                if (requirement->GetCurrentCommand() != nullptr)
                {
                    requirement->GetCurrentCommand()->end(true);
                }
                (*isDependentSubsystem)->SetCurrentCommand(control);
            }
        }
        commandList.insert(control);
        control->initialize();
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

    bool CommandScheduler::isScheduled(Command* command)
    {
        return commandList.find(command) != commandList.end();
    }

    void CommandScheduler::registerSubsystem(Subsystem* subsystem)
    {
        auto subsystemListItr1 = subsystemList.find(subsystem);
        if (subsystemListItr1 == subsystemList.end())
        {
            subsystemList.insert(subsystem);
        }
    }
}  // namespace control

}  // namespace aruwlib
