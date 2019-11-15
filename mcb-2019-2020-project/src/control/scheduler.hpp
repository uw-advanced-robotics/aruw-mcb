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

#include "rm-dev-board-a/board.hpp"
#include "src/control/command.hpp"
#include <modm/container/linked_list.hpp>
#include "src/control/subsystem.hpp"
#include "src/algorithms/include/bits/stl_set.h"

namespace aruwlib
{
 
namespace control
{

class Scheduler
{
 public:
    static void run(void);

    // smart pointer, memory allocation
    static bool addCommand(aruwlib::control::Command* command);

    static void removeCommand(Command* command);

    static bool isScheduled(Command* command);

    static void registerSubsystem(Subsystem* subsystem);

 private:
    static std::set<Subsystem*> subsystemList1;

    static std::set<Command*> commandList1;

   //  static modm::LinkedList<Command*> commandList;

   //  static modm::LinkedList<Subsystem*> subsystemList;
};

}  // namespace control

}  // namespace aruwlib

#endif
