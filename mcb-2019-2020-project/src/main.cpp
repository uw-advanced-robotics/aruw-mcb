#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>
#include <modm/processing/timer.hpp>

#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"
#include "aruwlib/communication/remote.hpp"

#include "aruwsrc/control/chassis_subsystem.hpp"
#include "aruwsrc/control/chassis_drive_command.hpp"
#include "aruwsrc/control/turret_subsystem.hpp"
#include "src/aruwlib/algorithms/contiguous_float_test.hpp"

using namespace aruwsrc::control;

ChassisSubsystem soldierChassis;

int main()
{
    Board::initialize();
    aruwlib::Remote::initialize();

    modm::SmartPointer chassisDrive(new ChassisDriveCommand(&soldierChassis));

    CommandScheduler::registerSubsystem(&soldierChassis);

    CommandScheduler::addCommand(chassisDrive);

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
