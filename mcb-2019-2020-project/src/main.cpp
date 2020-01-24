#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>
#include <modm/processing/timer.hpp>

#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"

#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwsrc/control/example_command.hpp"
#include "src/aruwsrc/control/example_subsystem.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"
#include "src/aruwlib/algorithms/contiguous_float_test.hpp"
#include "src/aruwsrc/control/engineer/engineer_17mm_reservoir_subsystem.hpp"
#include "src/aruwsrc/control/engineer/engineer_17mm_reservoir_rotate_command.hpp"

aruwsrc::control::Engineer17mmReservoirSubsystem engineer17mmReservoir;

using namespace aruwlib::sensors;
using namespace aruwsrc::control;

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

    //Mpu6500::init();

    // reservoir rotate testing
    aruwlib::control::CommandScheduler::registerSubsystem(&engineer17mmReservoir);

    while (!engineer17mmReservoir.reservoirCalibrateHere())
    {
        aruwlib::can::CanRxHandler::pollCanData();
        modm::delayMilliseconds(1);
    }

    // rotate in 3rds taking two seconds for a rotation segment
    modm::SmartPointer rotateCommand(new Engineer17mmReservoirRotateCommand(&engineer17mmReservoir, 2.0f * aruwlib::algorithms::PI / 3, 2000));

    //engineer17mmReservoir.setDefaultCommand(rotateCommand);

    CommandScheduler::addCommand(rotateCommand);

    //modm::SmartPointer testDefaultCommand(
    //    new aruwsrc::control::ExampleCommand(&testSubsystem));

    //testSubsystem.setDefaultCommand(testDefaultCommand);

    //CommandScheduler::registerSubsystem(&testSubsystem);

    // timers
    // arbitrary, taken from last year since this send time doesn't overfill
    // can bus
    modm::ShortPeriodicTimer motorSendPeriod(3);
    // update imu
    modm::ShortPeriodicTimer updateImuPeriod(2);

    while (1)
    {
        // do this as fast as you can
        aruwlib::can::CanRxHandler::pollCanData();

        /*
        if (updateImuPeriod.execute())
        {
            Mpu6500::read();
        }
        */

        if (motorSendPeriod.execute())
        {
            //motorSendPeriod.restart(3);
            //aruwlib::control::CommandScheduler::run();
            engineer17mmReservoir.refresh();
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }

        modm::delayMicroseconds(10);
    }
    return 0;
}
