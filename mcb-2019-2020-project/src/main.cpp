#include <rm-dev-board-a/board.hpp>
#include "src/control/scheduler.hpp"

#include "src/control/command-example.hpp"
#include "src/control/subsystem-example.hpp"

aruwsrc::control::SubsystemExample frictionWheelSystem(8, 0, 0, 10000);
aruwsrc::control::CommandExample frictionWheelDefaultCommand(&frictionWheelSystem);

int main()
{
    frictionWheelDefaultCommand.schedule();
    
    Board::initialize();

    while (1)
    {
        aruwlib::control::Scheduler::run();
        modm::delayMicroseconds(100);
    }
    return 0;
}
