#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>
#include <modm/processing/timer.hpp>

#include "src/control/command_scheduler.hpp"
#include "src/control/example_command.hpp"
#include "src/control/example_subsystem.hpp"
#include "src/motor/dji_motor_tx_handler.hpp"
#include "src/communication/can/can_rx_listener.hpp"
#include "src/aruwsrc/control/agitator_subsystem.hpp"
#include "src/aruwsrc/control/agitator_rotate_command.hpp"
#include "src/algorithms/math_user_utils.hpp"
#include <modm/processing/timer.hpp>
#include "src/aruwsrc/control/shoot_comprised_command.hpp"
#include "src/aruwsrc/control/agitator_unjam_command.hpp"

using namespace aruwsrc::control;

AgitatorSubsystem agitator17mm(36);
ExampleSubsystem frictionWheelSubsystem;

int main()
{
    Board::initialize();

    modm::SmartPointer spinFrictionWheelCommand(new ExampleCommand(&frictionWheelSubsystem));
    frictionWheelSubsystem.setDefaultCommand(spinFrictionWheelCommand);

    aruwlib::control::CommandScheduler::registerSubsystem(&agitator17mm);
    CommandScheduler::registerSubsystem(&frictionWheelSubsystem);

    modm::ShortPeriodicTimer t(2);

    while (!agitator17mm.agitatorCalibrateHere())
    {
        aruwlib::can::CanRxHandler::pollCanData();
        modm::delayMilliseconds(1);
    }

    bool pressed = false;

    modm::SmartPointer unjamCommand(new AgitatorUnjamCommand(&agitator17mm, PI));
    modm::SmartPointer rotateCommand(new AgitatorRotateCommand(&agitator17mm, PI / 5));
    modm::SmartPointer shootCommand(new ShootComprisedCommand(&agitator17mm, PI / 5, PI / 2));

    while (1)
    {
        aruwlib::can::CanRxHandler::pollCanData();

        if (Board::Button::read() && !pressed)
        {
            aruwlib::control::CommandScheduler::addCommand(shootCommand);
        }

        pressed = Board::Button::read();

        // run scheduler
        if (t.execute())
        {
            t.restart(2);
            aruwlib::control::CommandScheduler::run();
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }

        // do this as fast as you can
        aruwlib::can::CanRxHandler::pollCanData();

        modm::delayMicroseconds(10);
    }
    return 0;
}
