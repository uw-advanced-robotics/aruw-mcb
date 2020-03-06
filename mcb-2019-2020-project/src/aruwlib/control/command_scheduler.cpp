#include <utility>
#include <set>
#include <algorithm>
#include <modm/processing/timer.hpp>
#include "command_scheduler.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_handler.hpp"
#include "command.hpp"
#include "src/aruwlib/errors/create_errors.hpp"

using namespace std;

namespace aruwlib
{

namespace control
{
    uint32_t CommandScheduler::commandSchedulerTimestamp = 0;

    CommandScheduler CommandScheduler::mainScheduler;

    CommandScheduler& CommandScheduler::getMainScheduler()
    {
        return mainScheduler;
    }

    void CommandScheduler::addCommand(Command* commandToAdd)
    {
        if (commandToAdd == nullptr)
        {
            RAISE_ERROR("attempting to add nullptr command",
                    aruwlib::errors::Location::COMMAND_SCHEDULER,
                    aruwlib::errors::ErrorType::ADDING_NULLPTR_COMMAND);
            return;
        }

        bool commandAdded = false;

        const set<Subsystem*>& commandRequirements = commandToAdd->getRequirements();
        // end all commands running on the subsystem requirements.
        // They were interrupted.
        // Additionally, replace the current command with the commandToAdd
        for (auto& requirement : commandRequirements)
        {
            map<Subsystem*, Command*>::iterator subsystemRequirementCommandPair =
                subsystemToCommandMap.find(requirement);
            if (subsystemRequirementCommandPair != subsystemToCommandMap.end())
            {
                if (subsystemRequirementCommandPair->second != nullptr)
                {
                    subsystemRequirementCommandPair->second->end(true);
                }
                subsystemRequirementCommandPair->second = commandToAdd;
                commandAdded = true;
            }
            else
            {
                // the command you are trying to add has a subsystem that is not in the
                // scheduler, so you cannot add it (will lead to undefined control behavior)
                RAISE_ERROR("Attempting to add a command without subsystem in the scheduler",
                        aruwlib::errors::Location::COMMAND_SCHEDULER,
                        aruwlib::errors::ErrorType::RUN_TIME_OVERFLOW);
                return;
            }
        }

        // initialize the commandToAdd. Only do this once even though potentially
        // multiple subsystems rely on this command.
        if (commandAdded)
        {
            commandToAdd->initialize();
        }
    }

    uint32_t fme = 0;
    int iiii;

    void CommandScheduler::run()
    {
        uint32_t checkRunPeriod = Board::getTimeMicroseconds();

        // Timestamp for reference and for disallowing a command from running
        // multiple times during the same call to run.
        if (this == &mainScheduler)
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
                currSubsystemCommandPair.first->prevSchedulerExecuteTimestamp
                    = commandSchedulerTimestamp;
            }
        }
        // make sure we are not going over tolerable runtime, otherwise something is really
        // wrong with the code
        fme = Board::getTimeMicroseconds() - checkRunPeriod;
        if (Board::getTimeMicroseconds() - checkRunPeriod > MAX_ALLOWABLE_SCHEDULER_RUNTIME)
        {
            // shouldn't take more than 1 ms to complete all this stuff, if it does something
            // is seriously wrong (i.e. you are adding subsystems unchecked)
            std::string s = std::to_string(Board::getTimeMicroseconds() - checkRunPeriod);
            RAISE_ERROR("scheduler took longer than",
                    aruwlib::errors::Location::COMMAND_SCHEDULER,
                    aruwlib::errors::ErrorType::RUN_TIME_OVERFLOW);
        }
    }

    void CommandScheduler::removeCommand(Command* command, bool interrupted)
    {
        if (command == nullptr)
        {
            return;
        }
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

    bool CommandScheduler::isCommandScheduled(Command* command) const
    {
        if (command == nullptr)
        {
            return false;
        }
        return std::any_of(subsystemToCommandMap.begin(), subsystemToCommandMap.end(),
            [command](pair<Subsystem*, Command*> p)
            {
                return p.second == command;
            }
        );
    }

    void CommandScheduler::registerSubsystem(Subsystem* subsystem)
    {
        if (subsystem != nullptr && !isSubsystemRegistered(subsystem))
        {
            subsystemToCommandMap[subsystem] = nullptr;
        } else {
            RAISE_ERROR("subsystem is already added or trying to add nullptr subsystem",
                    aruwlib::errors::Location::COMMAND_SCHEDULER,
                    aruwlib::errors::ErrorType::ADDING_NULLPTR_COMMAND);
        }
    }

    bool CommandScheduler::isSubsystemRegistered(Subsystem* subsystem) const
    {
        if (subsystem == nullptr)
        {
            return false;
        }
        return subsystemToCommandMap.find(subsystem) != subsystemToCommandMap.end();
    }
}  // namespace control

}  // namespace aruwlib
