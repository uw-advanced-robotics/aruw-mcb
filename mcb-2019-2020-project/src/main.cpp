#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>
#include <modm/processing/timer.hpp>

#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwsrc/control/example_command.hpp"
#include "src/aruwsrc/control/example_subsystem.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"

#include "src/aruwlib/algorithms/contiguous_float_test.hpp"
#include "src/aruwlib/property_sys.hpp"

aruwsrc::control::ExampleSubsystem testSubsystem;
aruwlib::PropertySystem propertySystem;
uint32_t a = 0;
uint16_t array[6] = {1, 2, 3, 4, 5, 6};
uint8_t n[5] = "test";
uint8_t n1[6] = "test1";
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
    propertySystem.initializePropertySystem();
    propertySystem.addProperty(&a, n, 5);
    propertySystem.addProperty(array, 6, n1, 6);
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
        // if (motorSendPeriod.execute())
        // {
        //     aruwlib::control::CommandScheduler::run();
        //     aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        // }

        // do this as fast as you can
        // aruwlib::can::CanRxHandler::pollCanData();

        propertySystem.updatePropertySystem();
        propertySystem.updateSerial();
        a += 1;
        array[0] += 1;
        array[2] += 1;
        array[3] += 1;
        modm::delayMicroseconds(10);
    }
    return 0;
}
