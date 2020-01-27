#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>
#include <modm/processing/timer.hpp>
#include <modm/processing/timer.hpp>

/* communication includes ---------------------------------------------------*/
#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"
#include "src/aruwlib/communication/remote.hpp"

/* math includes ------------------------------------------------------------*/
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/algorithms/contiguous_float_test.hpp"

/* aruwlib control includes -------------------------------------------------*/
#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwlib/control/controller_mapper.hpp"

/* aruwsrc control includes -------------------------------------------------*/
#include "src/aruwsrc/control/example_command.hpp"
#include "src/aruwsrc/control/example_comprised_command.hpp"
#include "src/aruwsrc/control/example_subsystem.hpp"
#include "src/aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "src/aruwsrc/control/agitator/shoot_steady_comprised_command.hpp"
#include "src/aruwsrc/control/agitator/agitator_calibrate_command.hpp"
#include "src/aruwsrc/control/agitator/agitator_shoot_comprised_commands.hpp"

using namespace aruwsrc::agitator;
using namespace aruwsrc::control;
using namespace aruwlib::algorithms;
using namespace aruwlib::sensors;
using namespace aruwlib;

/* main scheduler responsible for interfacing with user and cv input --------*/
aruwlib::control::CommandScheduler mainScheduler(true);

/* define subsystems --------------------------------------------------------*/
#if defined(TARGET_SOLDIER)
AgitatorSubsystem agitator17mm;
ExampleSubsystem frictionWheelSubsystem;
#endif

/* define commands ----------------------------------------------------------*/
#if defined(TARGET_SOLDIER)
aruwsrc::control::ExampleCommand spinFrictionWheelCommand(&frictionWheelSubsystem);
// ShootSlowComprisedCommand agitatorShootSlowCommand(&agitator17mm);

#include "src/aruwsrc/control/agitator/agitator_rotate_command.hpp"
AgitatorRotateCommand agitatorShootSlowCommand(&agitator17mm, PI / 5, 100, 100);


AgitatorCalibrateCommand agitatorCalibrateCommand(&agitator17mm);
#endif

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

    /* register subsystems here ---------------------------------------------*/
    #if defined(TARGET_SOLDIER)
    mainScheduler.registerSubsystem(&agitator17mm);
    mainScheduler.registerSubsystem(&frictionWheelSubsystem);
    #endif

    /* set any default commands to subsystems here --------------------------*/
    #if defined(TARGET_SOLDIER)
    frictionWheelSubsystem.setDefaultCommand(&spinFrictionWheelCommand);
    #endif

    /* add any starting commands to the scheduler here ----------------------*/
    #if defined(TARGET_SOLDIER)
    mainScheduler.addCommand(&agitatorCalibrateCommand);
    #endif

    /* define timers here ---------------------------------------------------*/
    modm::ShortPeriodicTimer updateImuPeriod(2);
    modm::ShortPeriodicTimer sendMotorTimeout(2);

    /* register io mappings here --------------------------------------------*/
    #if defined(TARGET_SOLDIER)
    IoMapper::addHoldRepeatMapping(
        IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP),
        &agitatorShootSlowCommand
    );
    #endif

    bool pressed = false;

    while (1)
    {
        // if (Remote::getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP
        //     && agitatorShootSlowCommand.isFinished()
        // ) {
        //     mainScheduler.addCommand(dynamic_cast<Command*>(&agitatorShootSlowCommand));
        // } else if (Remote::getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP
        //     && !pressed)
        // {
        //     mainScheduler.addCommand(dynamic_cast<Command*>(&agitatorShootSlowCommand));
        // }

        // pressed = Remote::getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP;



        can::CanRxHandler::pollCanData();

        Remote::read();

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
