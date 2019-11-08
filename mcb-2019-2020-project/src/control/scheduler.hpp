/**
 * Class for handling all the commands you would like to currently runs.
 * Only knows how to run commands and nothing else.
 * Contains list of all commands that need to be run currently, runs these
 * commands every time run is called. Uses isFinished function from command
 * to determine if a command should be completed. Unlike FRC, currently 
 * there subsystems do not have periodic functions that are called in
 * the scheduler. Instead, commands handle subsystem updates when
 * necessary.
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
#include <modm/container/dynamic_array.hpp>
#include <modm/container/queue.hpp>

namespace aruwlib
{

namespace control
{

class Scheduler
{
 public:
    static void run(void);

    static bool addCommand(aruwlib::control::Command* command); // smart pointer, memory allocation

    static void removeCommand(const Command* command);

    static void resetAll(void);

    static bool isScheduled(const Command* command);

    static bool addSubsystem(Subsystem* subsystem);
    
    static void motorSendReceiveRatio(uint16_t motorSendReceiveRatio);

 private:
    static modm::LinkedList<Command*> commandList;

    static modm::LinkedList<Subsystem*> subsystemList;

    static uint16_t sendReceiveRatio;

    static uint16_t updateCounter;
};

}  // namespace control

}  // namespace aruwlib

#endif
