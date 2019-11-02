/**
 * Class for handling all the commands you would like to currently runs.
 * Only knows how to run commands and nothing else.
 * Contains list of all commands that need to be run currently, runs these
 * commands every time run is called. Uses isFinished function from command
 * to determine if a command should be completed. Unlike FRC, currently 
 * there subsystems do not have periodic functions that are called in
 * the scheduler. Instead, commands handle subsystem updates when
 * necessary. 
 */

#ifndef __SCHEDULER_HPP__
#define __SCHEDULER_HPP__

#include "rm-dev-board-a/board.hpp"
#include "src/control/command.hpp"
#include <modm/container/linked_list.hpp>
#include "src/control/subsystem.hpp"

namespace aruwlib
{

namespace control
{

class Scheduler
{
 public:
    static void addCommand(aruwlib::control::Command* command); // smart pointer, memory allocation

    static void run(void);

    static void removeCommand(Command* command);

    static void resetAll(void);

    static bool isScheduled(const Command* command);

 private:
    static modm::LinkedList<Command*> commandList;
};

}  // namespace control

}  // namespace aruwlib

#endif
