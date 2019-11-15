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
#include <set>
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

    static bool addCommand(aruwlib::control::Command* command);

    static void removeCommand(Command* command);

    static bool isCommandScheduled(Command* command);

    static void registerSubsystem(Subsystem* subsystem);

    static bool isSubsystemRegistered(const Subsystem* subsystem);

 private:
    static std::set<Subsystem*> subsystemList;

    static std::set<Command*> commandList;
};

}  // namespace control

}  // namespace aruwlib

#endif
