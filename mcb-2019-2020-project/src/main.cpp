#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>
#include <modm/processing/timer.hpp>

#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"
#include "aruwlib/communication/remote.hpp"

#include "aruwsrc/control/chassis_subsystem.hpp"
#include "aruwsrc/control/chassis_drive_command.hpp"
#include "aruwsrc/control/chassis_autorotate_command.hpp"
#include "aruwsrc/control/example_subsystem.hpp"
#include "aruwsrc/control/example_command.hpp"
#include "aruwsrc/control/turret_subsystem.hpp"
#include "src/aruwlib/algorithms/contiguous_float_test.hpp"

using namespace aruwsrc::control;

ChassisSubsystem soldierChassis;
//TurretSubsystem soldierTurret;
//ExampleSubsystem frictionWheelSubsystem;

int main()
{
    /*
    aruwlib::algorithms::ContiguousFloatTest contiguousFloatTest;
    contiguousFloatTest.testCore();
    contiguousFloatTest.testBadBounds();
    contiguousFloatTest.testDifference();
    contiguousFloatTest.testRotationBounds();
    contiguousFloatTest.testShiftingValue();
    contiguousFloatTest.testWrapping();
    */

    Board::initialize();
    aruwlib::Remote::initialize();

    /*
    modm::SmartPointer chassisAutorotate(
        new ChassisAutorotateCommand(&soldierChassis, &soldierTurret));
    modm::SmartPointer frictionWheelSpinCommand(new ExampleCommand(&frictionWheelSubsystem));
    */

    modm::SmartPointer chassisDrive(new ChassisDriveCommand(&soldierChassis));

    CommandScheduler::registerSubsystem(&soldierChassis);
    //CommandScheduler::registerSubsystem(&soldierTurret);
    //CommandScheduler::registerSubsystem(&frictionWheelSubsystem);

    CommandScheduler::addCommand(chassisDrive);
    //CommandScheduler::addCommand(chassisAutorotate);
    //CommandScheduler::addCommand(frictionWheelSpinCommand);

    // timers
    // arbitrary, taken from last year since this send time doesn't overfill
    // can bus
    modm::ShortPeriodicTimer motorSendPeriod(3);

    while (1)
    {
        aruwlib::Remote::read();
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
