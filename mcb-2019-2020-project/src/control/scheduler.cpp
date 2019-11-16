#include <utility>
#include "src/control/scheduler.hpp"
#include "src/motor/dji_motor_tx_handler.hpp"
#include "src/communication/can/can_rx_handler.hpp"

using namespace std;

namespace aruwlib
{

namespace control
{
    map<Subsystem*, Command*> CommandScheduler::subsystemToCommandMap;

    uint32_t CommandScheduler::commandSchedulerTimestamp = 0;

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
        for (const Subsystem* requirement : commandToAdd->getRequirements())
        {
            auto isRequirementRegistered = subsystemToCommandMap.find(
                const_cast<Subsystem*>(requirement));

            if (isRequirementRegistered == subsystemToCommandMap.end()
                || (isRequirementRegistered->second != NULL
                && !isRequirementRegistered->second->isInterruptible())
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

    void CommandScheduler::run()
    {
        // timestamp for reference and for disallowing a command from running
        // multiple times during the same call to run
        commandSchedulerTimestamp++;
        // refresh all and run all commands
        for (auto& currSubsystemCommandPair : subsystemToCommandMap) {
            // add default command if no command is currently being run
            if (currSubsystemCommandPair.second == NULL
                && currSubsystemCommandPair.first->GetDefaultCommand() != nullptr) {
                addCommand(currSubsystemCommandPair.first->GetDefaultCommand());
            }
            // only run the command if it hasn't been run this time run has been called
            if (currSubsystemCommandPair.second != NULL)
            {
                if (currSubsystemCommandPair.second->prevSchedulerExecuteTimestamp
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
            }
            // refresh subsystem
            currSubsystemCommandPair.first->refresh();
        }
    }

    void CommandScheduler::removeCommand(Command* command)
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

    bool CommandScheduler::isCommandScheduled(Command* command)
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

    bool CommandScheduler::registerSubsystem(Subsystem* subsystem)
    {
        if (!isSubsystemRegistered(subsystem))
        {
            subsystemToCommandMap.insert(pair<Subsystem*, Command*>(subsystem, NULL));
            return true;
        }
        return false;
    }

    bool CommandScheduler::isSubsystemRegistered(Subsystem* subsystem)
    {
        return subsystemToCommandMap.find(subsystem) != subsystemToCommandMap.end();
    }
}  // namespace control

}  // namespace aruwlib
