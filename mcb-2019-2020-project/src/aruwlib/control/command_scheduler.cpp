#include <utility>
#include <set>
#include <algorithm>
#include <modm/processing/timer.hpp>
#include "command_scheduler.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_handler.hpp"
#include "comprised_command.hpp"

using namespace std;

namespace aruwlib
{

namespace control
{
    const float CommandScheduler::MAX_ALLOWABLE_SCHEDULER_RUNTIME = 0.5f;

    const modm::SmartPointer CommandScheduler::defaultNullCommand(0);

    map<Subsystem*, modm::SmartPointer> CommandScheduler::subsystemToCommandMap;
    // map<Subsystem*, Command*> CommandScheduler::subsystemToCommandMap;

    modm::LinkedList<modm::SmartPointer> CommandScheduler::comprisedCommandList;

    uint32_t CommandScheduler::commandSchedulerTimestamp = 0;

    bool CommandScheduler::addCommand(modm::SmartPointer commandToAdd)
    {
        // only add the command if (a) command is not already being run and (b) all
        // subsystem dependencies can be interrupted.
        if (isCommandScheduled(commandToAdd))
        {
            return false;
        }

        bool commandAdded = false;

        set<Subsystem*> commandRequirements =
            *(smrtPtrCommandCast(commandToAdd)->getRequirements());
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
                    smrtPtrCommandCast(isDependentSubsystem->second)->end(true);
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
            smrtPtrCommandCast(commandToAdd)->initialize();
        }
        return true;
    }

    bool CommandScheduler::addComprisedCommand(modm::SmartPointer comprisedCommand)
    {
        // remove the command if it is already added
        for (int i = 0; i < static_cast<int>(comprisedCommandList.getSize()); i++)
        {
            modm::SmartPointer addedComprisedCommand = comprisedCommandList.getFront();
            if (addedComprisedCommand == comprisedCommand)
            {
                smrtPtrCommandCast(addedComprisedCommand)->end(true);
            }
            else
            {
                comprisedCommandList.append(addedComprisedCommand);
            }
        }

        set<Subsystem*> commandRequirements =
            *smrtPtrCommandCast(comprisedCommand)->getRequirements();
        for (auto& requirement : commandRequirements)
        {
            // return false if the command you are trying to add has a subsystem that is not in the
            // command scheduler
            map<Subsystem*, modm::SmartPointer>::iterator isDependentSubsystem =
                subsystemToCommandMap.find(requirement);
            if (isDependentSubsystem == subsystemToCommandMap.end())
            {
                return false;
            }
        }

        comprisedCommandList.append(comprisedCommand);
        // don't add commands here, for a comprised command, you add them yourselves
        smrtPtrCommandCast(comprisedCommand)->initialize();
        return true;
    }

    void CommandScheduler::run()
    {
        uint32_t checkRunPeriod = DWT->CYCCNT;  // clock cycle count
        // timestamp for reference and for disallowing a command from running
        // multiple times during the same call to run
        commandSchedulerTimestamp++;
        // refresh all comprised commands
        for (int i = 0; i < static_cast<int>(comprisedCommandList.getSize()); i++)
        {
            modm::SmartPointer comprisedCommand = comprisedCommandList.getFront();
            comprisedCommandList.removeFront();
            Command* currComprisedCommand = smrtPtrCommandCast(comprisedCommand);
            currComprisedCommand->execute();

            if (currComprisedCommand->isFinished())
            {
                currComprisedCommand->end(false);
            }
            else
            {
                comprisedCommandList.append(comprisedCommand);
            }
        }
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
                Command* currCommand = smrtPtrCommandCast(currSubsystemCommandPair.second);

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
                    currSubsystemCommandPair.second = defaultNullCommand;
                }
            }
            // refresh subsystem
            currSubsystemCommandPair.first->refresh();
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

    void CommandScheduler::removeCommand(modm::SmartPointer command, bool interrupted)
    {
        bool commandFound = false;
        for (auto& subsystemCommandPair : subsystemToCommandMap)
        {
            if (subsystemCommandPair.second == command)
            {
                if (!commandFound)
                {
                    smrtPtrCommandCast(subsystemCommandPair.second)->end(interrupted);
                    commandFound = true;
                }
                subsystemCommandPair.second = defaultNullCommand;
            }
        }
    }

    void CommandScheduler::removeComprisedCommand(
        const modm::SmartPointer& comprisedCommand,
        bool interrupted
    ) {
        for (int i = 0; i < static_cast<int>(comprisedCommandList.getSize()); i++)
        {
            auto command = comprisedCommandList.getFront();
            comprisedCommandList.removeFront();
            if (!(command == comprisedCommand))
            {
                comprisedCommandList.append(comprisedCommand);
            }
            else
            {
                smrtPtrCommandCast(comprisedCommand)->end(interrupted);
            }
        }
    }

    bool CommandScheduler::isCommandScheduled(modm::SmartPointer command)
    {
        return std::any_of(subsystemToCommandMap.begin(), subsystemToCommandMap.end(),
            [command](pair<Subsystem*, modm::SmartPointer> p)
            {
                return p.second == command;
            }
        );
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

    Command* CommandScheduler::smrtPtrCommandCast(modm::SmartPointer smrtPtr)
    {
        return reinterpret_cast<Command*>(smrtPtr.getPointer());
    }
}  // namespace control

}  // namespace aruwlib
