#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>
#include <modm/processing/timer.hpp>

#include "aruwlib/control/command_scheduler.hpp"
#include "aruwlib/motor/dji_motor_tx_handler.hpp"
#include "aruwlib/communication/can/can_rx_listener.hpp"
#include "aruwlib/communication/remote.hpp"

#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "aruwlib/algorithms/contiguous_float_test.hpp"

using namespace aruwsrc::chassis;

#if defined(TARGET_SOLDIER)
ChassisSubsystem soldierChassis;
#endif

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

    // define subsystems/commands below

    #if defined(TARGET_SOLDIER)  // only soldier has the proper constants in for chassis code
    modm::SmartPointer chassisDrive(new ChassisDriveCommand(&soldierChassis));
    CommandScheduler::registerSubsystem(&soldierChassis);
    soldierChassis.setDefaultCommand(chassisDrive);
    #endif

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
