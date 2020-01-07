#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>
#include <modm/processing/timer.hpp>

#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"
#include "aruwlib/communication/remote.hpp"

#include "aruwsrc/control/chassis_subsystem.hpp"
#include "aruwsrc/control/chassis_autorotate_command.hpp"
#include "aruwsrc/control/turret_subsystem.hpp"
 
using namespace aruwsrc::control;

ChassisSubsystem soldierChassis;
TurretSubsystem soldierTurret;

float turretOffset = 0.0f;

int main()
{
    Board::initialize();
    aruwlib::Remote::initialize();
    ChassisAutorotateCommand car(&soldierChassis, &soldierTurret);
    modm::SmartPointer chassisAutorotate(&car);
        // new ChassisAutorotateCommand(&soldierChassis, &soldierTurret));
    CommandScheduler::registerSubsystem(&soldierChassis);
    CommandScheduler::registerSubsystem(&soldierTurret);
    CommandScheduler::addCommand(chassisAutorotate);

    // timers
    // arbitrary, taken from last year since this send time doesn't overfill
    // can bus
    modm::ShortPeriodicTimer motorSendPeriod(3);

    while (1)
    {
        aruwlib::Remote::read();
        if (motorSendPeriod.execute())
        {
            turretOffset = soldierTurret.gimbalGetOffset();
            // car.execute();
            // soldierChassis.refresh();
            aruwlib::control::CommandScheduler::run();
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }

        // do this as fast as you can
        aruwlib::can::CanRxHandler::pollCanData();

        modm::delayMicroseconds(10);
    }
    return 0;
}
