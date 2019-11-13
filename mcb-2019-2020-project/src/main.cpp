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



aruwsrc::control::CommandExample CommandFrictionWheelDefault(&subsystemFrictionWheel);

aruwlib::motor::DjiMotor* m3;

using namespace std;

Subsystem* s2;

int main()
{
    subsystemFrictionWheel.SetDefaultCommand(&CommandFrictionWheelDefault);
    Scheduler::registerSubsystem(&subsystemFrictionWheel);

    Board::initialize();

    s2 = &subsystemFrictionWheel;

    while (1)
    {
        aruwlib::control::Scheduler::run();
        modm::delayMicroseconds(10);

        modm::delayMilliseconds(1000);
        Board::Leds::toggle();
    }
    return 0;
}
