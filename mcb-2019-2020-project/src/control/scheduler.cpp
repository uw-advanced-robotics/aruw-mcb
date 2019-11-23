#include <utility>
#include "src/control/scheduler.hpp"
#include "src/motor/dji_motor_tx_handler.hpp"
#include "src/communication/can/can_rx_handler.hpp"

using namespace std;

namespace aruwlib
{

namespace control
{
    map<Subsystem*, modm::SmartPointer> CommandScheduler::subsystemToCommandMap;
    // map<Subsystem*, Command*> CommandScheduler::subsystemToCommandMap;

    uint32_t CommandScheduler::commandSchedulerTimestamp = 0;

    const modm::SmartPointer CommandScheduler::defaultNullCommand(0);

    bool CommandScheduler::addCommand(modm::SmartPointer commandToAdd)
    {
        // only add the command if (a) command is not already being run and (b) all
        // subsystem dependencies can be interrupted.
        if (isCommandScheduled(commandToAdd))
        {
            return false;
        }

        bool commandAdded = false;

        set<Subsystem*> commandRequirements = *(getCmdPtr(commandToAdd)->getRequirements());
        // end all commands running on the subsystem requirements.
        // They were interrupted.
        // Additionally, replace the current command with the commandToAdd
        for (auto& requirement : commandRequirements)
        {
            map<Subsystem*, modm::SmartPointer>::iterator isDependentSubsystem =
                subsystemToCommandMap.find(requirement);
            if (isDependentSubsystem != subsystemToCommandMap.end())
            {
                if (!(isDependentSubsystem->second == defaultNullCommand))
                {
                    getCmdPtr(isDependentSubsystem->second)->end(true);
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
            getCmdPtr(commandToAdd)->initialize();
        }
        return true;
    }

    void CommandScheduler::run()
    {
        // timestamp for reference and for disallowing a command from running
        // multiple times during the same call to run
        commandSchedulerTimestamp++;
        // refresh all and run all commands
        for (auto& currSubsystemCommandPair : subsystemToCommandMap)
        {
            // add default command if no command is currently being run
            if (currSubsystemCommandPair.second == defaultNullCommand
                && !(currSubsystemCommandPair.first->getDefaultCommand() == defaultNullCommand)
            ){
                addCommand(currSubsystemCommandPair.first->getDefaultCommand());
            }
            // only run the command if it hasn't been run this time run has been called
            if (!(currSubsystemCommandPair.second == defaultNullCommand))
            {
                Command* currCommand = getCmdPtr(currSubsystemCommandPair.second);

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
                    currSubsystemCommandPair.second = 0;
                }
            }
            // refresh subsystem
            currSubsystemCommandPair.first->refresh();
        }
    }

    void CommandScheduler::removeCommand(modm::SmartPointer command)
    {
        for (auto& subsystemCommandPair : subsystemToCommandMap)
        {
            if (subsystemCommandPair.second == command)
            {
                subsystemCommandPair.second = defaultNullCommand;
            }
        }
    }

    bool CommandScheduler::isCommandScheduled(modm::SmartPointer command)
    {
        for (pair<Subsystem*, modm::SmartPointer> subsystemCommandPair : subsystemToCommandMap)
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
            subsystemToCommandMap[subsystem] = defaultNullCommand;
            return true;
        }
        return false;
    }

    bool CommandScheduler::isSubsystemRegistered(Subsystem* subsystem)
    {
        return subsystemToCommandMap.find(subsystem) != subsystemToCommandMap.end();
    }

    Command* CommandScheduler::getCmdPtr(modm::SmartPointer smrtPtr)
    {
        return reinterpret_cast<Command*>(smrtPtr.getPointer());
    }
}  // namespace control

}  // namespace aruwlib
