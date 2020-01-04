#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>
#include <modm/processing/timer.hpp>

#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwsrc/control/example_command.hpp"
#include "src/aruwsrc/control/example_subsystem.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"
#include "src/aruwsrc/control/agitator_subsystem.hpp"
#include "src/aruwsrc/control/agitator_rotate_command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include <modm/processing/timer.hpp>
#include "src/aruwsrc/control/shoot_comprised_command.hpp"
#include "src/aruwsrc/control/agitator_unjam_command.hpp"
#include "src/aruwlib/algorithms/contiguous_float_test.hpp"

#include "src/aruwlib/communication/remote.hpp"

using namespace aruwsrc::control;
using namespace aruwlib::algorithms;
using namespace aruwlib;

AgitatorSubsystem agitator17mm(36);
ExampleSubsystem frictionWheelSubsystem;

bool pressed = true;

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

    modm::SmartPointer spinFrictionWheelCommand(new ExampleCommand(&frictionWheelSubsystem));
    frictionWheelSubsystem.setDefaultCommand(spinFrictionWheelCommand);

    aruwlib::control::CommandScheduler::registerSubsystem(&agitator17mm);
    CommandScheduler::registerSubsystem(&frictionWheelSubsystem);

    modm::ShortPeriodicTimer t(2);

    while (!agitator17mm.agitatorCalibrateHere())
    {
        aruwlib::can::CanRxHandler::pollCanData();
        modm::delayMilliseconds(1);
    }

    modm::SmartPointer unjamCommand(new AgitatorUnjamCommand(&agitator17mm, aruwlib::algorithms::PI));
    modm::SmartPointer rotateCommand(new AgitatorRotateCommand(&agitator17mm, aruwlib::algorithms::PI / 5));
    modm::SmartPointer shootCommand(new ShootComprisedCommand(&agitator17mm, aruwlib::algorithms::PI / 5, aruwlib::algorithms::PI / 2));

    while (1)
    {
        can::CanRxHandler::pollCanData();

        if (Remote::getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP)
        {
            control::CommandScheduler::addCommand(shootCommand);
        } else if (Remote::getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP
            && !pressed)
        {
            control::CommandScheduler::addCommand(shootCommand);
        }

        pressed = Remote::getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP;

        // run scheduler
        if (t.execute())
        {
            Remote::read();
            t.restart(2);
            control::CommandScheduler::run();
            motor::DjiMotorTxHandler::processCanSendData();
        }

        // do this as fast as you can
        can::CanRxHandler::pollCanData();

        modm::delayMicroseconds(10);
    }
    return 0;
}
