#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>
#include <modm/processing/timer.hpp>
#include <modm/processing/timer.hpp>

// communication includes
#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"
#include "src/aruwlib/communication/remote.hpp"

// math includes
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/algorithms/contiguous_float_test.hpp"

// aruwlib control includes
#include "src/aruwlib/control/command_scheduler.hpp"

// aruwsrc control includes
#include "src/aruwsrc/control/example_command.hpp"
#include "src/aruwsrc/control/example_comprised_command.hpp"
#include "src/aruwsrc/control/example_subsystem.hpp"
#include "src/aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "src/aruwsrc/control/agitator/agitator_rotate_command.hpp"
#include "src/aruwsrc/control/agitator/shoot_steady_comprised_command.hpp"
#include "src/aruwsrc/control/agitator/agitator_unjam_command.hpp"

using namespace aruwsrc::agitator;
using namespace aruwsrc::control;
using namespace aruwlib::algorithms;
using namespace aruwlib::sensors;
using namespace aruwlib;

aruwlib::control::CommandScheduler mainScheduler(true);

// define subsystems
AgitatorSubsystem agitator17mm;
ExampleSubsystem frictionWheelSubsystem;

// define commands
aruwsrc::control::ExampleCommand spinFrictionWheelCommand(&frictionWheelSubsystem);
ShootSteadyComprisedCommand agitatorShootCommand(
    &agitator17mm,
    aruwlib::algorithms::PI / 5.0f, 300.0f,
    aruwlib::algorithms::PI / 2.0f
);

int main()
{
    aruwlib::algorithms::ContiguousFloatTest contiguousFloatTest;
    contiguousFloatTest.testCore();
    contiguousFloatTest.testBadBounds();
    contiguousFloatTest.testDifference();
    contiguousFloatTest.testRotationBounds();
    contiguousFloatTest.testShiftingValue();
    contiguousFloatTest.testWrapping();

    Board::initialize();

    aruwlib::Remote::initialize();
    
    Mpu6500::init();

    mainScheduler.registerSubsystem(&agitator17mm);
    mainScheduler.registerSubsystem(&frictionWheelSubsystem);

    frictionWheelSubsystem.setDefaultCommand(&spinFrictionWheelCommand);

    modm::ShortPeriodicTimer t(2);

    while (!agitator17mm.agitatorCalibrateHere())  // have a calibrate command
    {
        aruwlib::can::CanRxHandler::pollCanData();
        modm::delayMilliseconds(1);
    }

    bool pressed = false;

    while (1)
    {
        can::CanRxHandler::pollCanData();

        Remote::read();

        if (Remote::getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP
            && agitatorShootCommand.isFinished()
        ) {
            mainScheduler.addCommand(reinterpret_cast<Command*>(&agitatorShootCommand));
        } else if (Remote::getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP
            && !pressed)
        {
            mainScheduler.addCommand(reinterpret_cast<Command*>(&agitatorShootCommand));
        }

        pressed = Remote::getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP;

        modm::delayMicroseconds(10);
    }
    return 0;
}
