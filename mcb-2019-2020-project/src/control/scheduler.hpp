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
#include <set>
#include <modm/container/linked_list.hpp>
#include <modm/container/smart_pointer.hpp>
#include "rm-dev-board-a/board.hpp"
#include "src/control/command.hpp"

namespace aruwlib
{

namespace control
{

class CommandScheduler
{
 public:
    static void run(void);

    void removeCommand(modm::SmartPointer command);

    static bool registerSubsystem(Subsystem* subsystem);

    static bool isSubsystemRegistered(Subsystem* subsystem);

    static bool isSubsystemRegistered(const Subsystem* subsystem);

    static bool isCommandScheduled(modm::SmartPointer command);

    static bool addCommand(modm::SmartPointer commandToAdd);

 private:
   //  static std::map<Subsystem*, Command*> subsystemToCommandMap;

    static std::map<Subsystem*, modm::SmartPointer> subsystemToCommandMap;

    static uint32_t commandSchedulerTimestamp;

    static Command* getCmdPtr(modm::SmartPointer smrtPtr);
};

}  // namespace control

}  // namespace aruwlib

#endif
