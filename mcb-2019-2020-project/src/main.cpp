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

using namespace aruwsrc::control;

// ExampleSubsystem frictionWheelSubsystem;
AgitatorSubsystem agitator17mm(36);

aruwlib::motor::DjiMotor* m3;

float f = 0.0f;

using namespace std;

aruwsrc::control::ExampleCommand* commandWatch;

int main()
{
    Board::initialize();

    // create new commands
    // modm::SmartPointer rotateAgitator(
    //     new AgitatorRotateCommand(&agitator17mm, PI / 2.0f)
    // );

    // add commands when necessary
    // aruwlib::control::CommandScheduler::addCommand(rotateAgitator);

    aruwlib::control::CommandScheduler::registerSubsystem(&agitator17mm);

    modm::ShortPeriodicTimer t(2);

    while (!agitator17mm.agitatorCalibrateHere())
    {
        aruwlib::can::CanRxHandler::pollCanData();
        modm::delayMilliseconds(1);
    }

    bool pressed = false;

    while (1)
    {
        f = agitator17mm.getAgitatorEncoderToPosition();
        aruwlib::can::CanRxHandler::pollCanData();

        if (!pressed && Board::Button::read())
        {
            modm::SmartPointer rotateCommand(new AgitatorRotateCommand(&agitator17mm, PI));
            aruwlib::control::CommandScheduler::addCommand(rotateCommand);
            // agitator17mm.agitatorCalibrateHere();
        }

        pressed = Board::Button::read();

        // run scheduler
        if (t.execute())
        {
            t.restart(2);
            aruwlib::control::CommandScheduler::run();
            // aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }

        // do this as fast as you can
        aruwlib::can::CanRxHandler::pollCanData();

        modm::delayMicroseconds(10);
    }
    return 0;
}
