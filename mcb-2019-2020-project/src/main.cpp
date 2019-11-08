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

int main()
{
    frictionWheelSystem.SetDefaultCommand(&frictionWheelDefaultCommand);
    Scheduler::addSubsystem(&frictionWheelSystem);
    Scheduler::motorSendReceiveRatio(30); // send every 3 ms, assuming 100 microsecond delay

    Board::initialize();

    while (1)
    {
        aruwlib::control::Scheduler::run();
        modm::delayMicroseconds(100);
    }
    return 0;
}
