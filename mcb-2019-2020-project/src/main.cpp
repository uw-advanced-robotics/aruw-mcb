#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>

#include "src/control/scheduler.hpp"
#include "src/control/example_command.hpp"
#include "src/control/example_subsystem.hpp"
#include "src/motor/dji_motor_tx_handler.hpp"
#include "src/communication/can/can_rx_listener.hpp"

#include <modm/processing/timer.hpp>


/*
tests to complete:
running command framework with following parameters:
nothing scheudled
single subsystem, no command
single subsystem, single command
multiple subsystems, no commands
multiple subsystems, any command attached to any subsystem
*/

// #define NO_SUBSYSTEM_TEST
// #define SINGLE_SUBSYSTEM_NO_COMMAND
// #define SINGLE_SUBSYSTEM_SINGLE_COMMAND
// #define SINGLE_SUBSYSTEM_TWO_COMMANDS
#define SINGLE_COMMAND
// #define TWO_SUBSYSTEMS
// #define TWO_SUBSYSTEMS_TWO_COMMANDS
// #define SINGLE_SUBSYSTEM_REMOVE_ADD_COMMAND

#if defined (NO_SUBSYSTEM_TEST)
#elif defined (SINGLE_SUBSYSTEM_NO_COMMAND) || defined (SINGLE_SUBSYSTEM_SINGLE_COMMAND)
aruwsrc::control::ExampleSubsystem frictionWheelSubsystem;
#elif defined (SINGLE_SUBSYSTEM_TWO_COMMANDS) || defined (SINGLE_SUBSYSTEM_REMOVE_ADD_COMMAND)
aruwsrc::control::ExampleSubsystem frictionWheelSubsystem;
#elif defined (TWO_SUBSYSTEMS) || defined (TWO_SUBSYSTEMS_TWO_COMMANDS)
aruwsrc::control::ExampleSubsystem frictionWheelSubsystem;
aruwsrc::control::ExampleSubsystem frictionWheelSubsystemOther(
    aruwlib::motor::MOTOR1, aruwlib::motor::MOTOR2);
#endif

using namespace std;

aruwsrc::control::ExampleCommand* commandWatchTest;
const aruwlib::control::Subsystem* subsystemWatch;

int main()
{
    Board::initialize();

    #if defined (NO_SUBSYSTEM_TEST)
    #elif defined (SINGLE_SUBSYSTEM_NO_COMMAND)
    #elif defined (SINGLE_SUBSYSTEM_SINGLE_COMMAND) || defined (SINGLE_SUBSYSTEM_REMOVE_ADD_COMMAND)
    modm::SmartPointer frictionWheelDefaultCommand(
        new aruwsrc::control::ExampleCommand(&frictionWheelSubsystem));

    frictionWheelSubsystem.setDefaultCommand(frictionWheelDefaultCommand);

    commandWatchTest = reinterpret_cast<aruwsrc::control::ExampleCommand*>
        (frictionWheelDefaultCommand.getPointer());
    #elif defined (SINGLE_SUBSYSTEM_TWO_COMMANDS)
    modm::SmartPointer frictionWheelDefaultCommand(
        new aruwsrc::control::ExampleCommand(&frictionWheelSubsystem));
    modm::SmartPointer frictionWheelOtherCommand(
        new aruwsrc::control::ExampleCommand(&frictionWheelSubsystem));

    frictionWheelSubsystem.setDefaultCommand(frictionWheelDefaultCommand);

    #elif defined (SINGLE_COMMAND)
    modm::SmartPointer frictionWheelCommand(
        new aruwsrc::control::ExampleCommand());

    // try to add a command, you will not be able to do this
    CommandScheduler::addCommand(frictionWheelCommand);

    commandWatchTest = reinterpret_cast<aruwsrc::control::ExampleCommand*>
        (frictionWheelCommand.getPointer());

    #elif defined (TWO_SUBSYSTEMS_TWO_COMMANDS)
    modm::SmartPointer frictionWheelDefaultCommand(
        new aruwsrc::control::ExampleCommand(&frictionWheelSubsystem));
    modm::SmartPointer frictionWheelOtherCommand(
        new aruwsrc::control::ExampleCommand(&frictionWheelSubsystemOther));

    frictionWheelSubsystem.setDefaultCommand(frictionWheelDefaultCommand);
    #endif

    // timers
    modm::ShortPeriodicTimer motorSendPeriod(3);

    #if defined (SINGLE_SUBSYSTEM_TWO_COMMANDS) || defined (TWO_SUBSYSTEMS_TWO_COMMANDS)
    modm::ShortPeriodicTimer executeOtherCommand(100);
    #endif

    #if defined (SINGLE_SUBSYSTEM_REMOVE_ADD_COMMAND)
    bool buttonHigh = false;
    #endif

    while (1)
    {
        #if defined (SINGLE_SUBSYSTEM_REMOVE_ADD_COMMAND)
        if (Board::Button::read() && !buttonHigh)
        {
            CommandScheduler::removeCommand(frictionWheelDefaultCommand);
        }
        buttonHigh = Board::Button::read();
        #endif

        // where should this go?
        #if defined (SINGLE_SUBSYSTEM_TWO_COMMANDS) || defined (TWO_SUBSYSTEMS_TWO_COMMANDS)
        if (executeOtherCommand.execute())
            CommandScheduler::addCommand(frictionWheelOtherCommand);
        #endif

        if (motorSendPeriod.execute())
        {
            aruwlib::control::CommandScheduler::run();
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }

        // do this as fast as you can
        aruwlib::can::CanRxHandler::pollCanData();

        modm::delayMicroseconds(10);
    }
    return 0;
}
