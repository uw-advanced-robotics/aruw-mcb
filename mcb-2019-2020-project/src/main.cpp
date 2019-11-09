#include <rm-dev-board-a/board.hpp>
#include "src/control/scheduler.hpp"

#include "src/control/command-example.hpp"
#include "src/control/subsystem-example.hpp"

#include "src/motor/dji_motor_tx_handler.hpp"
#include "src/communication/can/can_rx_listener.hpp"

aruwsrc::control::SubsystemExample frictionWheelSystem(
    4, 0, 0, 5000,
    aruwlib::motor::MotorId::MOTOR5,
    aruwlib::motor::MotorId::MOTOR6
);

aruwsrc::control::CommandExample frictionWheelDefaultCommand(&frictionWheelSystem);

modm::DynamicArray<int>* i;
int ii;
int main()
{
    i = new modm::DynamicArray<int>(10);
    i->append(10);
    
    ii = i->operator[](0);
      //  frictionWheelSystem.SetDefaultCommand(&frictionWheelDefaultCommand);
    Scheduler::addSubsystem(&frictionWheelSystem);
    frictionWheelDefaultCommand.CommandExampleInit(&frictionWheelSystem);
    Scheduler::motorSendReceiveRatio(30); // send every 3 ms, assuming 100 microsecond delay

    frictionWheelDefaultCommand.schedule();

    Board::initialize();

    while (1)
    {
        aruwlib::control::Scheduler::run();
        //modm::delayMilliseconds(1);
        modm::delayMicroseconds(10);
    }
    return 0;
}
