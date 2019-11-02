#include <rm-dev-board-a/board.hpp>
#include "src/control/scheduler.hpp"

#include "src/control/command-example.hpp"
#include "src/control/subsystem-example.hpp"

#include "src/motor/dji_motor_tx_handler.hpp"
#include "src/communication/can/can_rx_listener.hpp"

aruwsrc::control::SubsystemExample frictionWheelSystem(
    8, 0, 0, 10000,
    aruwlib::motor::MotorId::MOTOR5,
    aruwlib::motor::MotorId::MOTOR6
);
aruwsrc::control::CommandExample frictionWheelDefaultCommand(&frictionWheelSystem);

int value;
bool result;
int main()
{
    frictionWheelSystem.SetDefaultCommand(&frictionWheelDefaultCommand);
    Scheduler::addSubsystem(&frictionWheelSystem);
    Board::initialize();

    while (1)
    {
        aruwlib::can::CanRxHandler::pollCanData();
        aruwlib::control::Scheduler::run();
        modm::delayMicroseconds(100);
    }
    return 0;
}
