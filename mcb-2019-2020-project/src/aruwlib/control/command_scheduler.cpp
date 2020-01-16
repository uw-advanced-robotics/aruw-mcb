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

    map<Subsystem*, pair<modm::SmartPointer, modm::SmartPointer>> CommandScheduler::subsystemToCommandMap;

    uint32_t CommandScheduler::commandSchedulerTimestamp = 0;

    bool CommandScheduler::addCommand(modm::SmartPointer commandToAdd)
    {
        // only add the command if (a) command is not already being run and (b) all
        // subsystem dependencies can be interrupted.
        if (isCommandScheduled(commandToAdd))
        {
            return false;
        }

        const set<Subsystem*> commandRequirements =
            *smrtPtrCommandCast(commandToAdd)->getRequirements();

        for (auto& requirement : commandRequirements)
        {
            if (subsystemToCommandMap.find(requirement) == subsystemToCommandMap.end())
            {
                return false;
            }
        }

        /*
         * we have already checked to make sure all the necessary subsystems are
         * accounted for, now we just have to add/remove the commands
         * end all commands running on the subsystem requirements.
         * They were interrupted.
         * Additionally, replace the current command with the commandToAdd
         */
        for (auto& requirement : commandRequirements)
        {
            const auto& isDependentSubsystem = subsystemToCommandMap.find(requirement);

            if (isDependentSubsystem != subsystemToCommandMap.end())
            {
                // check to see if there is a command associated with the subsystem
                if (!(isDependentSubsystem->second.first == defaultNullCommand))
                {
                    smrtPtrCommandCast(isDependentSubsystem->second.first)->end(true);
                }
                // do the same for ComprisedCommands unless the comprised command uses the command
                // i.e. the smart pointer matches
                if (
                    !(isDependentSubsystem->second.second == defaultNullCommand)
                    && !reinterpret_cast<ComprisedCommand*>(
                        isDependentSubsystem->second.second.getPointer())->
                        usesCommand(commandToAdd)
                ) {
                    smrtPtrCommandCast(isDependentSubsystem->second.second)->end(true);
                }
                isDependentSubsystem->second.first = commandToAdd;
            }
        }

        // initialize the commandToAdd. Only do this once even though potentially
        // multiple subsystems rely on this command.
        smrtPtrCommandCast(commandToAdd)->initialize();
        return true;
    }

    bool CommandScheduler::addComprisedCommand(modm::SmartPointer comprisedCommandToAdd)
    {
        return false;
    }

    void CommandScheduler::run()
    {
        uint32_t checkRunPeriod = DWT->CYCCNT;  // clock cycle count
        // timestamp for reference and for disallowing a command from running
        // multiple times during the same call to run
        commandSchedulerTimestamp++;
        // refresh all and run all commands
        for (auto& currSubsystemCommandPair : subsystemToCommandMap)
        {
            // add default command if no command is currently being run
            if (currSubsystemCommandPair.second.first == defaultNullCommand
                && !(currSubsystemCommandPair.first->getDefaultCommand() == defaultNullCommand)
            ){
                addCommand(currSubsystemCommandPair.first->getDefaultCommand());
            }
            // only run the command if it hasn't been run this time run has been called
            if (!(currSubsystemCommandPair.second.first == defaultNullCommand))
            {
                Command* currCommand = smrtPtrCommandCast(currSubsystemCommandPair.second.first);

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
                    currSubsystemCommandPair.second.first = defaultNullCommand;
                }
            }
            if (!(currSubsystemCommandPair.second.second == defaultNullCommand))
            {
                // todo dedupe this if we actually decide to use this version
                Command* currCommand = smrtPtrCommandCast(currSubsystemCommandPair.second.second);

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
                    currSubsystemCommandPair.second.second = defaultNullCommand;
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
            if (subsystemCommandPair.second.first == command)
            {
                if (!commandFound)
                {
                    smrtPtrCommandCast(subsystemCommandPair.second.first)->end(interrupted);
                    commandFound = true;
                }
                subsystemCommandPair.second.first = defaultNullCommand;
            }
        }
    }

    void CommandScheduler::removeComprisedCommand(
        const modm::SmartPointer& comprisedCommand,
        bool interrupted
    ) {
        bool commandFound = false;
        for (auto& subsystemCommandPair : subsystemToCommandMap)
        {
            if (subsystemCommandPair.second.second == comprisedCommand)
            {
                if (!commandFound)
                {
                    smrtPtrCommandCast(subsystemCommandPair.second.second)->end(interrupted);
                    commandFound = true;
                }
                subsystemCommandPair.second.second = defaultNullCommand;
            }
        }
    }

    bool CommandScheduler::isCommandScheduled(modm::SmartPointer command)
    {
        return std::any_of(subsystemToCommandMap.begin(), subsystemToCommandMap.end(),
            [command](pair<Subsystem*, pair<modm::SmartPointer, modm::SmartPointer>> p)
            {
                return p.second.first == command;
            }
        );
    }

    bool CommandScheduler::registerSubsystem(Subsystem* subsystem)
    {
        if (!isSubsystemRegistered(subsystem))
        {
            subsystemToCommandMap.insert(
                pair<Subsystem*, pair<modm::SmartPointer, modm::SmartPointer>>
                (subsystem, pair<modm::SmartPointer, modm::SmartPointer>
                    (defaultNullCommand, defaultNullCommand)
                )
            );
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
