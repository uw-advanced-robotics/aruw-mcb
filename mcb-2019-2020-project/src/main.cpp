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
#include "src/aruwlib/control/controller_mapper.hpp"

using namespace aruwsrc::agitator;
using namespace aruwsrc::control;
using namespace aruwlib::algorithms;
using namespace aruwlib::sensors;
using namespace aruwlib;

// main scheduler responsible for interfacing with user and cv input
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

    Mpu6500::init();

    // register subsystems here
    mainScheduler.registerSubsystem(&agitator17mm);
    mainScheduler.registerSubsystem(&frictionWheelSubsystem);

    // set any default commands here
    frictionWheelSubsystem.setDefaultCommand(&spinFrictionWheelCommand);

    // define timers here
    modm::ShortPeriodicTimer updateImuPeriod(2);
    modm::ShortPeriodicTimer sendMotorTimeout(2);

    while (!agitator17mm.agitatorCalibrateHere())  // todo have a calibrate command
    {
        aruwlib::can::CanRxHandler::pollCanData();
        modm::delayMilliseconds(1);
    }

    // bool pressed = false;

    while (1)
    {
        can::CanRxHandler::pollCanData();

        Remote::read();

        // if (Remote::getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP
        //     && agitatorShootCommand.isFinished()
        // ) {
        //     mainScheduler.addCommand(dynamic_cast<Command*>(&agitatorShootCommand));
        // } else if (Remote::getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP
        //     && !pressed)
        // {
        //     mainScheduler.addCommand(dynamic_cast<Command*>(&agitatorShootCommand));
        // }
        // pressed = Remote::getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP;

        if (updateImuPeriod.execute())
        {
            Mpu6500::read();
        }
        
        if (sendMotorTimeout.execute())
        {
            mainScheduler.run();
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }

        modm::delayMicroseconds(10);
    }
    return 0;
}
