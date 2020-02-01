#include <utility>
#include <set>
#include <algorithm>
#include <modm/processing/timer.hpp>
#include "command_scheduler.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_handler.hpp"
#include "command.hpp"

using namespace std;

namespace aruwlib
{

namespace control
{
    uint32_t CommandScheduler::commandSchedulerTimestamp = 0;

    bool CommandScheduler::addCommand(Command* commandToAdd)
    {
        bool commandAdded = false;

        const set<Subsystem*>& commandRequirements = commandToAdd->getRequirements();
        // end all commands running on the subsystem requirements.
        // They were interrupted.
        // Additionally, replace the current command with the commandToAdd
        for (auto& requirement : commandRequirements)
        {
            map<Subsystem*, Command*>::iterator isDependentSubsystem =
                subsystemToCommandMap.find(requirement);
            if (isDependentSubsystem != subsystemToCommandMap.end())
            {
                if (isDependentSubsystem->second != nullptr)
                {
                    isDependentSubsystem->second->end(true);
                }
                isDependentSubsystem->second = commandToAdd;
                commandAdded = true;
            }
            else
            {
                // the command you are trying to add has a subsystem that is not in the
                // scheduler, so you cannot add it (will lead to undefined control behavior)
                return false;
            }
        }

        // initialize the commandToAdd. Only do this once even though potentially
        // multiple subsystems rely on this command.
        if (commandAdded)
        {
            commandToAdd->initialize();
        }
        return true;
    }

    void CommandScheduler::run()
    {
        uint32_t checkRunPeriod = DWT->CYCCNT;  // clock cycle count
        // Timestamp for reference and for disallowing a command from running
        // multiple times during the same call to run.
        if (isMainScheduler)
        {
            commandSchedulerTimestamp++;
        }
        // refresh all and run all commands
        for (auto& currSubsystemCommandPair : subsystemToCommandMap)
        {
            // add default command if no command is currently being run
            if (currSubsystemCommandPair.second == nullptr
                && currSubsystemCommandPair.first->getDefaultCommand() != nullptr
            ){
                addCommand(currSubsystemCommandPair.first->getDefaultCommand());
            }
            // only run the command if it hasn't been run this time run has been called
            if (currSubsystemCommandPair.second != nullptr)
            {
                Command* currCommand = currSubsystemCommandPair.second;

                if (currCommand->prevSchedulerExecuteTimestamp
                    != commandSchedulerTimestamp
                ) {
                    currCommand->execute();
                    currCommand->prevSchedulerExecuteTimestamp
                        = commandSchedulerTimestamp;
                }
                // remove command if finished running
                if (currCommand->isFinished())
                {
                    currCommand->end(false);
                    currSubsystemCommandPair.second = nullptr;
                }
            }
            // refresh subsystem
            if (currSubsystemCommandPair.first->prevSchedulerExecuteTimestamp
                != commandSchedulerTimestamp) {
                currSubsystemCommandPair.first->refresh();
                currSubsystemCommandPair.first->prevSchedulerExecuteTimestamp = commandSchedulerTimestamp;
            }
        }
        // make sure we are not going over tolerable runtime, otherwise something is really
        // wrong with the code
        if (static_cast<float>(DWT->CYCCNT - checkRunPeriod)
            / static_cast<float>(modm::clock::fcpu_kHz)
            > MAX_ALLOWABLE_SCHEDULER_RUNTIME)
        {
            // shouldn't take more than 1 ms to complete all this stuff, if it does something
            // is seriously wrong (i.e. you are adding subsystems unchecked)
            // THROW-NON-FATAL-ERROR-CHECK
        }
    }

    void CommandScheduler::removeCommand(Command* command, bool interrupted)
    {
        bool commandFound = false;
        for (auto& subsystemCommandPair : subsystemToCommandMap)
        {
            if (subsystemCommandPair.second == command)
            {
                if (!commandFound)
                {
                    subsystemCommandPair.second->end(interrupted);
                    commandFound = true;
                }
                subsystemCommandPair.second = nullptr;
            }
        }
    }

    bool CommandScheduler::isCommandScheduled(Command* command)
    {
        return std::any_of(subsystemToCommandMap.begin(), subsystemToCommandMap.end(),
            [command](pair<Subsystem*, Command*> p)
            {
                return p.second == command;
            }
        );
    }

    bool CommandScheduler::registerSubsystem(Subsystem* subsystem)
    {
        if (!isSubsystemRegistered(subsystem))
        {
            subsystemToCommandMap[subsystem] = nullptr;
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
