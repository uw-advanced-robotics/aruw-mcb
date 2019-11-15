#include <rm-dev-board-a/board.hpp>
#include "src/control/scheduler.hpp"

#include "src/control/command-example.hpp"
#include "src/control/subsystem-example.hpp"

#include "src/motor/dji_motor_tx_handler.hpp"
#include "src/communication/can/can_rx_listener.hpp"

#include <modm/container/smart_pointer.hpp>

aruwsrc::control::SubsystemExample subsystemFrictionWheel(
    10, 0, 0, 0, 5000,
    aruwlib::motor::MotorId::MOTOR5,
    aruwlib::motor::MotorId::MOTOR6
);

aruwsrc::control::SubsystemExample subsystemFrictionWheel2(
    10, 0, 0, 0, 5000,
    aruwlib::motor::MotorId::MOTOR1,
    aruwlib::motor::MotorId::MOTOR2
);

aruwsrc::control::CommandExample CommandFrictionWheelDefault(&subsystemFrictionWheel);
aruwsrc::control::CommandExample CommandFrictionWheelDefault2(&subsystemFrictionWheel2);

aruwlib::motor::DjiMotor* m3;

using namespace std;

Subsystem* s2;
Subsystem* s3;

int main()
{
    subsystemFrictionWheel2.SetDefaultCommand(&CommandFrictionWheelDefault2);
    subsystemFrictionWheel.SetDefaultCommand(&CommandFrictionWheelDefault);
    CommandScheduler::registerSubsystem(&subsystemFrictionWheel);
    CommandScheduler::registerSubsystem(&subsystemFrictionWheel2);


    Board::initialize();

    s2 = &subsystemFrictionWheel;

    while (1)
    {
        aruwlib::control::CommandScheduler::run();
        modm::delayMicroseconds(10);

        modm::delayMilliseconds(1000);
        Board::Leds::toggle();
    }
    return 0;
}
