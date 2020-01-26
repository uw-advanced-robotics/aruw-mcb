#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>
#include <modm/processing/timer.hpp>

#include "src/aruwlib/control/controller_mapper.hpp"
#include "src/aruwsrc/control/blink_led_command.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"
#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwsrc/control/example_command.hpp"
#include "src/aruwsrc/control/example_comprised_command.hpp"
#include "src/aruwsrc/control/example_subsystem.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"
#include "src/aruwlib/algorithms/contiguous_float_test.hpp"

aruwsrc::control::ExampleSubsystem testSubsystem;

aruwlib::control::CommandScheduler mainScheduler(true);

// aruwsrc::control::ExampleCommand testDefaultCommand(&testSubsystem);
aruwsrc::control::ExampleComprisedCommand testComprisedCommand(&testSubsystem);
// aruwsrc::control::ExampleComprisedCommand test2(&testSubsystem);

aruwsrc::control::BlinkLEDCommand blinkCommand(&testSubsystem);

using namespace aruwlib::sensors;

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
    aruwlib::Remote::initialize();

    Mpu6500::init();

    mainScheduler.registerSubsystem(&testSubsystem);
    // mainScheduler.addCommand(&testDefaultCommand);
    mainScheduler.addCommand(&testComprisedCommand);

    // timers
    // arbitrary, taken from last year since this send time doesn't overfill
    // can bus
    modm::ShortPeriodicTimer motorSendPeriod(3);
    // update imu
    modm::ShortPeriodicTimer updateImuPeriod(2);

    IoMapper::addToggleMapping(
        IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP, {}),
        &blinkCommand
    );

    while (1)
    {
        // do this as fast as you can
        aruwlib::can::CanRxHandler::pollCanData();

        aruwlib::Remote::read();

        if (updateImuPeriod.execute())
        {
            Mpu6500::read();
        }

        if (motorSendPeriod.execute())
        {
            mainScheduler.run();
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }

        modm::delayMicroseconds(10);
    }
    return 0;
}
