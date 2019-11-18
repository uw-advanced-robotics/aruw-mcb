#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>

#include "src/control/scheduler.hpp"
#include "src/control/example-command.hpp"
#include "src/control/example-subsystem.hpp"
#include "src/motor/dji_motor_tx_handler.hpp"
#include "src/communication/can/can_rx_listener.hpp"

aruwsrc::control::ExampleSubsystem frictionWheelSubsystem;

aruwlib::motor::DjiMotor* m3;

using namespace std;

int count = 0;

aruwsrc::control::ExampleCommand* commandWatch;

int main()
{
    Board::initialize();

    modm::SmartPointer frictionWheelDefaultCommand(new aruwsrc::control::ExampleCommand(&frictionWheelSubsystem));
    frictionWheelSubsystem.SetDefaultCommand(frictionWheelDefaultCommand);
    commandWatch = reinterpret_cast<aruwsrc::control::ExampleCommand*>(frictionWheelDefaultCommand.getPointer());

    while (1)
    {
        aruwlib::can::CanRxHandler::pollCanData();
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
