#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>
#include <modm/processing/timer.hpp>

#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"

#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwsrc/control/example_command.hpp"
#include "src/aruwsrc/control/example_comprised_command.hpp"
#include "src/aruwsrc/control/example_subsystem.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"
#include "src/aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "src/aruwsrc/control/agitator/agitator_rotate_command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include <modm/processing/timer.hpp>
#include "src/aruwsrc/control/agitator/shoot_steady_comprised_command.hpp"
#include "src/aruwsrc/control/agitator/agitator_unjam_command.hpp"
#include "src/aruwlib/algorithms/contiguous_float_test.hpp"
#include "src/aruwlib/communication/remote.hpp"

using namespace aruwsrc::agitator;
using namespace aruwsrc::control;
using namespace aruwlib::algorithms;
using namespace aruwlib;

AgitatorSubsystem agitator17mm;
ExampleSubsystem frictionWheelSubsystem;

#include "src/aruwlib/algorithms/contiguous_float_test.hpp"

aruwsrc::control::ExampleSubsystem testSubsystem;

aruwlib::control::CommandScheduler mainScheduler(true);

// aruwsrc::control::ExampleCommand testDefaultCommand(&testSubsystem);
aruwsrc::control::ExampleComprisedCommand testComprisedCommand(&testSubsystem);

// aruwsrc::control::ExampleComprisedCommand test2(&testSubsystem);

using namespace aruwlib::sensors;

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
    
    //Mpu6500::init();

    // adding agitator for testing
    modm::SmartPointer spinFrictionWheelCommand(new ExampleCommand(&frictionWheelSubsystem));
    frictionWheelSubsystem.setDefaultCommand(spinFrictionWheelCommand);

    aruwlib::control::CommandScheduler::registerSubsystem(&agitator17mm);
    CommandScheduler::registerSubsystem(&frictionWheelSubsystem);

    modm::ShortPeriodicTimer t(2);

    // while (!agitator17mm.agitatorCalibrateHere())
    // {
    //     aruwlib::can::CanRxHandler::pollCanData();
    //     modm::delayMilliseconds(1);
    // }

    // modm::SmartPointer unjamCommand(new AgitatorUnjamCommand(&agitator17mm, aruwlib::algorithms::PI));
    // modm::SmartPointer rotateCommand(new AgitatorRotateCommand(&agitator17mm, aruwlib::algorithms::PI / 5));
    modm::SmartPointer shootCommand(new ShootSteadyComprisedCommand(&agitator17mm, aruwlib::algorithms::PI / 5, 300, aruwlib::algorithms::PI / 2));

    // CommandScheduler::addComprisedCommand(shootCommand);
    // CommandScheduler::removeComprisedCommand(shootCommand, false);
    // CommandScheduler::addComprisedCommand(shootCommand);
    // CommandScheduler::addComprisedCommand(shootCommand);
    // CommandScheduler::addCommand(spinFrictionWheelCommand);

    while (1)
    {
        can::CanRxHandler::pollCanData();

        Remote::read();

        if (Remote::getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP
            && CommandScheduler::smrtPtrCommandCast(shootCommand)->isFinished())
        {
            // control::CommandScheduler::addCommand(rotateCommand);
            control::CommandScheduler::addComprisedCommand(shootCommand);
        } else if (Remote::getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP
            && !pressed)
        {
                        // control::CommandScheduler::addCommand(rotateCommand);

            control::CommandScheduler::addComprisedCommand(shootCommand);
        }

        modm::delayMicroseconds(10);
    }
    return 0;
}
