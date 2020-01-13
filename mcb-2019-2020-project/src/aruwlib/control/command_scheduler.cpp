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

        /**
         * check comprised command requirements to see if any comprised commands need to
         * be removed. Only remove the requirement if the comprised command does not contain
         * the command that is trying to be added.
         */
        int comprisedCommandListSize = static_cast<int>(comprisedCommandList.getSize());
        for (int i = 0; i < comprisedCommandListSize; i++)
        {
            bool removeCommand = false;
            auto command = comprisedCommandList.getFront();
            comprisedCommandList.removeFront();
            set<Subsystem*> requirements = *smrtPtrCommandCast(commandToAdd)->getRequirements();
            for (auto& requirement : requirements)
            {
                if (smrtPtrCommandCast(command)->getRequirements()->find(requirement) !=
                    smrtPtrCommandCast(command)->getRequirements()->end()
                    && !reinterpret_cast<ComprisedCommand*>(command.getPointer())->
                        usesCommand(commandToAdd)
                ) {
                    removeCommand = true;
                    break;
                }
            }

            if (!removeCommand)
            {
                comprisedCommandList.append(command);
            }
            else
            {
                smrtPtrCommandCast(command)->end(true);
            }
        }

        const set<Subsystem*> commandRequirements =
            *smrtPtrCommandCast(commandToAdd)->getRequirements();
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

    bool CommandScheduler::addComprisedCommand(modm::SmartPointer comprisedCommandToAdd)
    {
        /**
         * review comprised command requirements
         * if a comprised command is being run that has any subsystem requirements that
         * are the same as the one trying to be added, end the comprised command
         * there is no need to end any command that are being run by the scheduler,
         * as this will happen in the below step
         * We do have to look at all subsystem dependencies, not just commands that are 
         * scheduled, since comprised commands could have subsystem dependencies but
         * the command being run currently does not yet tell the subsystem what to do
         */
        int initialComprisedCommandListSize = static_cast<int>(comprisedCommandList.getSize());
        for (int i = 0; i < initialComprisedCommandListSize; i++)
        {
            bool removeCurrCommand = false;
            // look at subsystem requirements. if overlapping subsystem requirements, remove
            // the currComprisedCommand
            modm::SmartPointer currComprisedCommand = comprisedCommandList.getFront();
            comprisedCommandList.removeFront();
            const set<Subsystem*> currCmdSubsystemRequirements =
                *smrtPtrCommandCast(currComprisedCommand)->getRequirements();
            const set<Subsystem*> cmdToAddSubsystemRequirements =
                *smrtPtrCommandCast(comprisedCommandToAdd)->getRequirements();
            for (auto subsystem : cmdToAddSubsystemRequirements)
            {
                if (currCmdSubsystemRequirements.find(subsystem)
                    != currCmdSubsystemRequirements.end()
                ) {
                    // shares a subsystem requirement, end the comprised command
                    removeCurrCommand = true;
                    break;
                }
            }
            if (!removeCurrCommand)  // add back to the list if we didn't remove
            {
                comprisedCommandList.append(currComprisedCommand);
            }
            else
            {
                smrtPtrCommandCast(currComprisedCommand)->end(true);
            }
        }

        /**
         * review subsystem requirements
         * if there is a subsystem that does not exist, you fail
         * if a subsystem has a command running, kill the command, but you don't fail
         */
        const set<Subsystem*> commandRequirements =
            *smrtPtrCommandCast(comprisedCommandToAdd)->getRequirements();
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
            else if (!(isDependentSubsystem->second == defaultNullCommand))
            {
                smrtPtrCommandCast(isDependentSubsystem->second)->end(true);
                isDependentSubsystem->second = defaultNullCommand;
            }
        }

        // you made it, add the comprised command. slack me a :jankturret
        // if you read this
        comprisedCommandList.append(comprisedCommandToAdd);
        // don't add commands here, for a comprised command, you add them yourselves
        smrtPtrCommandCast(comprisedCommandToAdd)->initialize();
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
        int size = static_cast<int>(comprisedCommandList.getSize());
        for (int i = 0; i < size; i++)
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
