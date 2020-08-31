#ifndef COMMAND_SCHEDULER_HPP_
#define COMMAND_SCHEDULER_HPP_

#include <algorithm>
#include <map>
#include <utility>

#include <modm/container/linked_list.hpp>

#include "aruwlib/architecture/clock.hpp"
#include "aruwlib/errors/create_errors.hpp"

#include "command.hpp"
#include "subsystem.hpp"

namespace aruwlib
{
namespace control
{
/**
 * Class for handling all the commands you would like to currently run.
 * Interfaces with the Subsystem and Command classes to provide a means
 * of safely scheduling multiple Commands and Subsystems. Checks are
 * provided while scheduling such that multiple commands that require
 * the same subsystem cannot run at the same time. Suppose for example
 * that you have a Command that moves a mechanical arm Subsystem to some
 * position and another Command that moves the same arm to a different
 * position. Obvious issues arise if one attempts to tell the Subsystem
 * to do two things at once. Using this class will disallow these two
 * Commands from being executed at the same time.
 *
 * This class contains a map of Subsystems -> Commands. The Subsystems
 * will be refreshed each time the CommandScheduler is ran. If there
 * are commands associated with the Subsystem, the CommandScheduler will
 * execute these commands as well. Additional less important features
 * are explained in more detail in the function definitions.
 *
 * The goal of this class is for the user to interace directly as
 * little as possible. Aside from calling `run` each time to update the
 * scheduler, the user should be interacting with the Command,
 * ComprisedCommand, Subsystem, and CommandMapper classes to add
 * and remove commands from the scheduler.
 *
 * The main use case will be to be refreshing all the main subsystems running
 * on the robot. To do so, you should call `getMainScheduler()` to access this
 * base scheduler. Here is an example of how to do this:
 *
 * ```
 * // A class that has Command as a base class.
 * CoolSubsystem sub;
 * // A class that has Subsystem as a base class that requires
 * // the subsystem above. In the constructor of the ControlCoolCommand,
 * // you must call `addSubsystemRequirement(dynamic_cast<Subsystem<Drivers>*>(subsystem))`;
 * ControlCoolCommand cmd(&sub);
 *
 * CommandScheduler::getMainScheduler().registerSubsystem(&sub);
 * CommandScheduler::getMainScheduler().addCommand(&cmd);
 *
 * while (1) {
 *     // The subsystem will refresh forever and the command until it is not finished.
 *     CommandScheduler::getMainScheduler().run();
 * }
 * ```
 *
 * The second use case for a CommandScheduler is in the ComprisedCommand class.
 * Here, you utilize the CommandScheduler to coordinate multiple commands inside a
 * single command. The usage is exactly the same as using the main CommandScheduler.
 */
template <typename Drivers>
class CommandScheduler
{
public:
    CommandScheduler() : subsystemToCommandMap() {}
    CommandScheduler(const CommandScheduler&) = delete;
    CommandScheduler& operator=(const CommandScheduler&) = delete;

    /**
     * Calls the `refresh()` function for all Subsystems and the associated
     * `execute()` function for all Commands. A Subsystem is guarenteed to
     * be refreshed no more than one time each time the mainScheduler's run
     * function is called. The same goes for a Command. This includes even if
     * multiple CommandSchedulers are running in ComprisedCommands that have
     * shared Subsystems.
     *
     * If any Subsystem that is in the scheduler does not have a Command
     * controlling it but does have a default command (via the Subsystem's
     * `getDefaultCommand()`), the default command is added to the scheduler.
     *
     * If any Command is finished after execution, the Command is removed from
     * the scheduler. The Command's `end()` function is called, passing in
     * `isInterrupted = false`.
     *
     * @note checks the run time of the scheduler. An error is added to the
     *      error handler if the time is greater than `MAX_ALLOWABLE_SCHEDULER_RUNTIME`
     *      (in microseconds).
     */
    void run()
    {
        uint32_t runStart = aruwlib::arch::clock::getTimeMicroseconds();
        // Timestamp for reference and for disallowing a command from running
        // multiple times during the same call to run.
        if (this == &Drivers::commandScheduler)
        {
            commandSchedulerTimestamp++;
        }
        // refresh all and run all commands
        for (auto& currSubsystemCommandPair : subsystemToCommandMap)
        {
            // add default command if no command is currently being run
            if (currSubsystemCommandPair.second == nullptr &&
                currSubsystemCommandPair.first->getDefaultCommand() != nullptr)
            {
                addCommand(currSubsystemCommandPair.first->getDefaultCommand());
            }
            // only run the command if it hasn't been run this time run has been called
            if (currSubsystemCommandPair.second != nullptr)
            {
                Command<Drivers>* currCommand = currSubsystemCommandPair.second;

                if (currCommand->prevSchedulerExecuteTimestamp != commandSchedulerTimestamp)
                {
                    currCommand->execute();
                    currCommand->prevSchedulerExecuteTimestamp = commandSchedulerTimestamp;
                }
                // remove command if finished running
                if (currCommand->isFinished())
                {
                    currCommand->end(false);
                    currSubsystemCommandPair.second = nullptr;
                }
            }
            // refresh subsystem
            if (currSubsystemCommandPair.first->prevSchedulerExecuteTimestamp !=
                commandSchedulerTimestamp)
            {
                currSubsystemCommandPair.first->refresh();
                currSubsystemCommandPair.first->prevSchedulerExecuteTimestamp =
                    commandSchedulerTimestamp;
            }
        }
        // make sure we are not going over tolerable runtime, otherwise something is really
        // wrong with the code
        uint32_t runEnd = aruwlib::arch::clock::getTimeMicroseconds();
        if (runEnd - runStart > MAX_ALLOWABLE_SCHEDULER_RUNTIME)
        {
            // shouldn't take more than 1 ms to complete all this stuff, if it does something
            // is seriously wrong (i.e. you are adding subsystems unchecked)
            RAISE_ERROR(
                "scheduler took longer than MAX_ALLOWABLE_SCHEDULER_RUNTIME",
                aruwlib::errors::Location::COMMAND_SCHEDULER,
                aruwlib::errors::ErrorType::RUN_TIME_OVERFLOW);
        }
    }

    /**
     * Removes the given Command completely from the CommandScheduler. This
     * means removing all instances of the command pointer from the Subsystem ->
     * Command map (since a single Subsystem can map to multiple Commands).
     *
     * @param[in] command the Command to remove. Must not be `nullptr`. If the
     *      Command is not in the scheduler, nothing is removed.
     * @param[in] interrupted an argument passed in to the Command's `end()`
     *      function when removing the desired Command.
     */
    void removeCommand(Command<Drivers>* command, bool interrupted)
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

    /**
     * Adds the given Subsystem to the CommandScheduler.  The subsystem is
     * added with the currently scheduled Command as `nullptr`.
     *
     * @param[in] subsystem the Subsystem to add. Must be not `nullptr` and not
     *      registered already (check via `isSubsystemRegistered()`), otherwise
     *      an error is added to the error handler.
     */
    void registerSubsystem(Subsystem<Drivers>* subsystem)
    {
        if (subsystem != nullptr && !isSubsystemRegistered(subsystem))
        {
            // Only initialize the subsystem when adding to main scheduler.
            if (this == &Drivers::commandScheduler)
            {
                subsystem->initialize();
            }
            subsystemToCommandMap[subsystem] = nullptr;
        }
        else
        {
            RAISE_ERROR(
                "subsystem is already added or trying to add nullptr subsystem",
                aruwlib::errors::Location::COMMAND_SCHEDULER,
                aruwlib::errors::ErrorType::ADDING_NULLPTR_COMMAND);
        }
    }

    /**
     * @param[in] subsystem the subsystem to check
     * @return `true` if the Subsystem is already scheduled, `false` otherwise.
     */
    bool isSubsystemRegistered(Subsystem<Drivers>* subsystem) const
    {
        if (subsystem == nullptr)
        {
            return false;
        }
        return subsystemToCommandMap.find(subsystem) != subsystemToCommandMap.end();
    }

    /**
     * @return `true` if the CommandScheduler contains the requrested Command.
     *      `false` otherwise.
     */
    bool isCommandScheduled(Command<Drivers>* command) const
    {
        if (command == nullptr)
        {
            return false;
        }
        return std::any_of(
            subsystemToCommandMap.begin(),
            subsystemToCommandMap.end(),
            [command](std::pair<Subsystem<Drivers>*, Command<Drivers>*> p) {
                return p.second == command;
            });
    }

    /**
     * Attempts to add a Command to the scheduler. There are a number of ways this
     * function can fail. If failure does occur, an error will be added to the
     * error handler.
     *
     * These are the following reasons why adding a Command fails:
     * - The commandToAdd is `nullptr`
     * - The commandToAdd has no Subsystem requirements.
     * - The commandToAdd has Subsystems not in the CommandScheduler.
     *
     * If a Command is successfully added to the CommandScheduler, any Subsystems
     * that the commandToAdd requires that have Commands running will be ended
     * (and the interrupted flag for that Command set to `true`).
     *
     * If a Command is successfully added, the Command's `initialize()` function will
     * be called.
     *
     * @param[in] commandToAdd the Command to be added to the scheduler.
     */
    void addCommand(Command<Drivers>* commandToAdd)
    {
        if (commandToAdd == nullptr)
        {
            RAISE_ERROR(
                "attempting to add nullptr command",
                aruwlib::errors::Location::COMMAND_SCHEDULER,
                aruwlib::errors::ErrorType::ADDING_NULLPTR_COMMAND);
            return;
        }

        bool commandAdded = false;

        const std::set<Subsystem<Drivers>*>& commandRequirements = commandToAdd->getRequirements();
        // end all commands running on the subsystem requirements.
        // They were interrupted.
        // Additionally, replace the current command with the commandToAdd
        for (auto& requirement : commandRequirements)
        {
            typename std::map<Subsystem<Drivers>*, Command<Drivers>*>::iterator
                subsystemRequirementCommandPair = subsystemToCommandMap.find(requirement);
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
                RAISE_ERROR(
                    "Attempting to add a command without subsystem in the scheduler",
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

private:
    ///< Maximum time before we start erroring, in microseconds.
    static constexpr float MAX_ALLOWABLE_SCHEDULER_RUNTIME = 100;

    ///< a map containing keys of subsystems, pairs of Commands.
    std::map<Subsystem<Drivers>*, Command<Drivers>*> subsystemToCommandMap;

    /**
     * @note this is not a true timestamp. Rather, we use this such that
     *      with multiple CommandSchedulers running with the same Subsystems,
     *      the Subsystems and Commands will be updated only once each time
     *      the `mainScheduler` is ran.
     */
    static uint32_t commandSchedulerTimestamp;
};  // class CommandScheduler

template <typename Drivers>
uint32_t CommandScheduler<Drivers>::commandSchedulerTimestamp = 0;

}  // namespace control

}  // namespace aruwlib

#endif  // COMMAND_SCHEDULER_HPP_
