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

#include <modm/processing/timer.hpp>

modm::ShortPeriodicTimer t(2);

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
    modm::SmartPointer rotateAgitator(
        new AgitatorRotateCommand(&agitator17mm, PI / 2.0f)
    );

    // add commands when necessary
    aruwlib::control::CommandScheduler::addCommand(rotateAgitator);

    bool prevRead = false;

    while (1)
    {
        f = agitator17mm.getAgitatorEncoderToPosition();
        aruwlib::can::CanRxHandler::pollCanData();

        // run scheduler
        if (t.execute())
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
