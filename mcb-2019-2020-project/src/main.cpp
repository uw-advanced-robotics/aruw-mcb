#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>

#include "src/control/scheduler.hpp"
#include "src/control/example-command.hpp"
#include "src/control/example-subsystem.hpp"
#include "src/motor/dji_motor_tx_handler.hpp"
#include "src/communication/can/can_rx_listener.hpp"
#include "src/aruwsrc/control/agitator_subsystem.hpp"
#include "src/aruwsrc/control/agitator_rotate_command.hpp"

using namespace aruwsrc::control;

ExampleSubsystem frictionWheelSubsystem;
AgitatorSubsystem agitator17mm(19);

aruwlib::motor::DjiMotor* m3;

using namespace std;

int count = 0;

aruwsrc::control::ExampleCommand* commandWatch;

int main()
{
    Board::initialize();

    // create new commands
    modm::SmartPointer frictionWheelDefaultCommand(
        new aruwsrc::control::ExampleCommand(&frictionWheelSubsystem));
    modm::SmartPointer rotateAgitator(
        new AgitatorRotateCommand(&agitator17mm, 60.0f)
    );

    // add commands if necessary
    frictionWheelSubsystem.SetDefaultCommand(frictionWheelDefaultCommand);

    commandWatch = reinterpret_cast<aruwsrc::control::ExampleCommand*>
        (frictionWheelDefaultCommand.getPointer());

    // add commands when necessary
    aruwlib::control::CommandScheduler::addCommand(rotateAgitator);

    while (1)
    {
        aruwlib::can::CanRxHandler::pollCanData();
        // run scheduler
        aruwlib::control::CommandScheduler::run();
        count++;
        if (count > 300)
        {
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
            count = 0;
        }
        modm::delayMicroseconds(10);
    }
    return 0;
}
