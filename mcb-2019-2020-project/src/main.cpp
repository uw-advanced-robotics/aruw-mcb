#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>

#include "src/control/scheduler.hpp"
#include "src/control/example-command.hpp"
#include "src/control/example-subsystem.hpp"
#include "src/motor/dji_motor_tx_handler.hpp"
#include "src/communication/can/can_rx_listener.hpp"

#include <modm/processing/timer.hpp>

aruwsrc::control::ExampleSubsystem frictionWheelSubsystem;

aruwlib::motor::DjiMotor* m3;

using namespace std;

aruwsrc::control::ExampleCommand* commandWatch;

int main()
{
    Board::initialize();

    // lots of random testing code

    modm::SmartPointer frictionWheelDefaultCommand(
        new aruwsrc::control::ExampleCommand(&frictionWheelSubsystem));

    frictionWheelSubsystem.SetDefaultCommand(frictionWheelDefaultCommand);
    commandWatch = reinterpret_cast<aruwsrc::control::ExampleCommand*>
        (frictionWheelDefaultCommand.getPointer());

    // timers
    modm::ShortPeriodicTimer motorSendPeriod(3);
    modm::ShortPeriodicTimer commandSchedulerRunPeriod(1);

    while (1)
    {
        // where should this go?
        if (motorSendPeriod.execute())
        {
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }

        // do this as fast as you can
        aruwlib::can::CanRxHandler::pollCanData();

        // testing
        if (commandSchedulerRunPeriod.execute())
        {
            aruwlib::control::CommandScheduler::run();
        }

        modm::delayMicroseconds(10);
    }
    return 0;
}
