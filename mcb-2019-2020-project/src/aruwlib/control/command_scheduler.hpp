/**
 * Class for handling all the commands you would like to currently runs.
 * Only knows how to run commands and refresh subsystems and nothing else.
 * 
 * Contains list of all commands and subsystems that need to be run
 * currently, runs these commands and refresh the subsystems  every time
 * run is called. Uses isFinished function from command to determine if
 * a command should be completed.
 *
 * The goal of this class is for the user to interace with this as little
 * as possible. Aside from run, the user should be interacting with the
 * command class and subsystem class to add and remove commands from the
 * sehcduler.
 */

#ifndef __SCHEDULER_HPP__
#define __SCHEDULER_HPP__

#include <map>
#include <modm/container/linked_list.hpp>
#include <modm/container/smart_pointer.hpp>
#include <rm-dev-board-a/board.hpp>

#include "subsystem.hpp"

namespace aruwlib
{

namespace control
{

class CommandScheduler
{
 public:
    CommandScheduler() : subsystemToCommandMap(), commandSchedulerTimestamp(0) {}

    void runCommands();

    void run();

    void removeCommand(modm::SmartPointer command, bool interrupted);

    bool registerSubsystem(Subsystem* subsystem);

    bool isSubsystemRegistered(Subsystem* subsystem);

    bool isSubsystemRegistered(const Subsystem* subsystem);

    bool isCommandScheduled(modm::SmartPointer command);

    bool addCommand(modm::SmartPointer commandToAdd);

    static const modm::SmartPointer defaultNullCommand;

    static Command* smrtPtrCommandCast(modm::SmartPointer smrtPtr);

 private:
    // maximum time before we start erroring, in seconds
    static constexpr float MAX_ALLOWABLE_SCHEDULER_RUNTIME = 0.5f;

    // a map containing keys of subsystems, pairs of Commands and ComprisedCommands
    // the command comes first, the ComprisedCommand second 
    std::map<Subsystem*, modm::SmartPointer> subsystemToCommandMap;

    uint32_t commandSchedulerTimestamp;
};

}  // namespace control

}  // namespace aruwlib

#endif
