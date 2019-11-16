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

#include <modm/container/linked_list.hpp>
#include <modm/container/smart_pointer.hpp>
#include <set>
#include "rm-dev-board-a/board.hpp"
#include "src/control/command.hpp"
#include <map>

namespace aruwlib
{
 
namespace control
{

class CommandScheduler
{
 public:
    static void run1(void);

    void removeCommand1(Command* command);

    static bool registerSubsystem1(Subsystem* subsystem);

    static bool isSubsystemRegistered1(Subsystem* subsystem);

    static bool isSubsystemRegistered1(const Subsystem* subsystem);

    static bool isCommandScheduled1(Command* command);

    static bool addCommand1(Command* commandToAdd);

 private:
    static std::map<Subsystem*, Command*> subsystemToCommandMap;

    static uint32_t commandSchedulerTimestamp;
};

}  // namespace control

}  // namespace aruwlib

#endif
