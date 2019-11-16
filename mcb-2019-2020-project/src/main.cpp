#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>

#include "src/control/scheduler.hpp"
#include "src/control/example-command.hpp"
#include "src/control/example-subsystem.hpp"
#include "src/motor/dji_motor_tx_handler.hpp"
#include "src/communication/can/can_rx_listener.hpp"

aruwsrc::control::ExampleSubsystem frictionWheelSubsystem(
    10, 0, 0, 0, 5000,
    aruwlib::motor::MotorId::MOTOR5,
    aruwlib::motor::MotorId::MOTOR6
);

aruwsrc::control::ExampleSubsystem frictionWheelSubsystem2(
    10, 0, 0, 0, 5000,
    aruwlib::motor::MotorId::MOTOR1,
    aruwlib::motor::MotorId::MOTOR2
);

aruwsrc::control::ExampleCommand frictionWheelDefaultCommand(&frictionWheelSubsystem);
aruwsrc::control::ExampleCommand frictionWheelDefaultCommand2(&frictionWheelSubsystem2);

aruwlib::motor::DjiMotor* m3;

using namespace std;

Subsystem* s2;
Subsystem* s3;

Command* c;
Command* c1;

int main()
{
    frictionWheelSubsystem2.SetDefaultCommand(&frictionWheelDefaultCommand2);
    frictionWheelSubsystem.SetDefaultCommand(&frictionWheelDefaultCommand);

    Board::initialize();

    s2 = &frictionWheelSubsystem;
    c = &frictionWheelDefaultCommand;
    c1 = &frictionWheelDefaultCommand2;

    while (1)
    {
        aruwlib::control::CommandScheduler::run();

        aruwlib::control::CommandScheduler::run1();

        modm::delayMicroseconds(10);

        modm::delayMilliseconds(1000);
        Board::Leds::toggle();
    }
    return 0;
}
