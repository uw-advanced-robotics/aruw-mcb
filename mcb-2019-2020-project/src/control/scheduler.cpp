#include "src/control/scheduler.hpp"
#include "src/motor/dji_motor_tx_handler.hpp"
#include "src/communication/can/can_rx_handler.hpp"

using namespace std;

namespace aruwlib
{

namespace control
{
    set<Subsystem*> CommandScheduler::subsystemList;

    set<modm::SmartPointer> subsystemList1;

    set<Command*> CommandScheduler::commandList;
    
    map<Subsystem*, Command*> CommandScheduler::subsystemToCommandMap;

    uint32_t CommandScheduler::commandSchedulerTimestamp = 0;

    bool CommandScheduler::addCommand1(Command* commandToAdd)
    {
        // only add the command if (a) command is not already being run and (b) all
        // subsystem dependencies can be interrupted.
        if (isCommandScheduled1(commandToAdd))
        {
            return false;
        }

        // Check to make sure the commandToAdd you are trying to add to the scheduler
        // can be added.
        // If there are command dependencies that can't be interrupted, don't schedule.
        for (auto& requirement : commandToAdd->getRequirements())
        {
            if (!isSubsystemRegistered1(requirement)
                || (requirement->GetCurrentCommand() != nullptr // this is wrong fix
                && !(requirement->GetCurrentCommand()->isInterruptible()))
            ) {
                return false;
            }
        }

        // end all commands running on the subsystem requirements.
        // They were interrupted.
        // Additionally, replace the current command with the commandToAdd
        for (auto& requirement : commandToAdd->getRequirements())
        {
            map<Subsystem*, Command*>::iterator isDependentSubsystem =
                subsystemToCommandMap.find(const_cast<Subsystem*>(requirement));
            if (isDependentSubsystem != subsystemToCommandMap.end())
            {
                if (isDependentSubsystem->second != NULL)
                {
                    isDependentSubsystem->second->end(true);
                }
                isDependentSubsystem->second = commandToAdd;
            }
        }

        // initialize the commandToAdd. Only do this once even though potentially
        // multiple subsystems rely on this command.
        commandToAdd->initialize();
        return true;
    }

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
                || (requirement->GetCurrentCommand() != nullptr
                && !(requirement->GetCurrentCommand()->isInterruptible()))
            ) {
                return false;
            }
        }

        // end all commands running on the subsystem requirements. 
        for (auto requirement : commandToAdd->getRequirements())
        {
            set<Subsystem*>::iterator isDependentSubsystem = subsystemList.find(
                const_cast<Subsystem*>(requirement));
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

    void CommandScheduler::run1()
    {
        // timestamp for reference and for disallowing a command from running
        // multiple times during the same call to run
        commandSchedulerTimestamp++;
        // refresh all and run all commands
        for (auto& currSubsystemCommandPair : subsystemToCommandMap) {
            // add default command if no command is currently being run
            if (currSubsystemCommandPair.first->GetCurrentCommand() == nullptr
                && currSubsystemCommandPair.first->GetDefaultCommand() != nullptr) {
                addCommand1(currSubsystemCommandPair.first->GetDefaultCommand());
            }
            // only run the command if it hasn't been run this time run has been called
            if (currSubsystemCommandPair.second != NULL
                && currSubsystemCommandPair.second->prevSchedulerExecuteTimestamp
                != commandSchedulerTimestamp
            ) {
                currSubsystemCommandPair.second->execute();
                currSubsystemCommandPair.second->prevSchedulerExecuteTimestamp
                    = commandSchedulerTimestamp;
            }
            // remove command if finished running
            if (currSubsystemCommandPair.second->isFinished())
            {
                currSubsystemCommandPair.second->end(false);
                currSubsystemCommandPair.second = NULL;
            }
            // refresh subsystem
            currSubsystemCommandPair.first->refresh();
        }
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

    void CommandScheduler::removeCommand1(Command* command)
    {
        for (auto subsystemCommandPair = subsystemToCommandMap.begin();
            subsystemCommandPair != subsystemToCommandMap.end();)
        {
            if (subsystemCommandPair->second == command)
            {
                subsystemCommandPair->second = NULL;
            }
        }
    }

    bool CommandScheduler::isSubsystemRegistered(const Subsystem* subsystem)
    {
        return subsystemList.find(const_cast<Subsystem*>(subsystem)) != subsystemList.end();
    }

    bool CommandScheduler::isSubsystemRegistered1(const Subsystem* subsystem)
    {
        return subsystemToCommandMap.find(const_cast<Subsystem*>(subsystem)) != subsystemToCommandMap.end();
    }

    bool CommandScheduler::isCommandScheduled(Command* command)
    {
        return commandList.find(command) != commandList.end();
    }

    bool CommandScheduler::isCommandScheduled1(Command* command)
    {
        for (pair<Subsystem*, Command*> subsystemCommandPair : subsystemToCommandMap)
        {
            if (subsystemCommandPair.second == command)
            {
                return true;
            }
        }
        return false;
    }

    void CommandScheduler::registerSubsystem(Subsystem* subsystem)
    {
        if (!isSubsystemRegistered(subsystem))
        {
            subsystemList.insert(subsystem);
        }
    }

    bool CommandScheduler::registerSubsystem1(Subsystem* subsystem)
    {
        if (!isSubsystemRegistered1(subsystem))
        {
            subsystemToCommandMap.insert(pair<Subsystem*, Command*>(subsystem, NULL));
            return true;
        }
        return false;
    }

    bool CommandScheduler::isSubsystemRegistered1(Subsystem* subsystem)
    {
        return subsystemToCommandMap.find(subsystem) != subsystemToCommandMap.end();
    }

}  // namespace control

}  // namespace aruwlib
