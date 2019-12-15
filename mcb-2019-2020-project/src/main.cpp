#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>
#include <modm/processing/timer.hpp>

#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"

#include "aruwsrc/control/chassis_subsystem.hpp"
#include "aruwsrc/control/chassis_autorotate_command.hpp"
 
using namespace aruwsrc::control;

ChassisSubsystem soldierChassis;

int main()
{
    Board::initialize();

    modm::SmartPointer chassisAutorotate(new ChassisAutorotateCommand(&soldierChassis));

    soldierChassis.setDefaultCommand(chassisAutorotate);

    CommandScheduler::registerSubsystem(&soldierChassis);

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
