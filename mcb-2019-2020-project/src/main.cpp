#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>
#include <modm/processing/timer.hpp>

#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwsrc/control/example_command.hpp"
#include "src/aruwsrc/control/example_subsystem.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"

#include "src/aruwlib/algorithms/contiguous_float_test.hpp"

#include "src/aruwsrc/control/example_comprised_command.hpp"

aruwsrc::control::ExampleSubsystem testSubsystem;

int main()
{
    aruwlib::algorithms::ContiguousFloatTest contiguousFloatTest;
    contiguousFloatTest.testCore();
    contiguousFloatTest.testBadBounds();
    contiguousFloatTest.testDifference();
    contiguousFloatTest.testRotationBounds();
    contiguousFloatTest.testShiftingValue();
    contiguousFloatTest.testWrapping();

    Board::initialize();

    modm::SmartPointer comprisedCommand(
        new aruwsrc::control::ExampleComprisedCommand(&testSubsystem));

    CommandScheduler::addComprisedCommand(comprisedCommand);

    modm::SmartPointer testDefaultCommand(
        new aruwsrc::control::ExampleCommand(&testSubsystem));

    testSubsystem.setDefaultCommand(testDefaultCommand);

    CommandScheduler::registerSubsystem(&testSubsystem);

    // timers
    // arbitrary, taken from last year since this send time doesn't overfill
    // can bus
    modm::ShortPeriodicTimer motorSendPeriod(3);

    while (1)
    {
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
