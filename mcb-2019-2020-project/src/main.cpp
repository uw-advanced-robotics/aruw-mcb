#include <rm-dev-board-a/board.hpp>
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
#include "src/aruwlib/communication/serial/ref_serial.hpp"
#include "src/aruwsrc/control/example_comprised_command.hpp"
#include "src/aruwlib/communication/serial/xavier_serial.hpp"

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
#include "src/aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "src/aruwsrc/control/chassis/chassis_subsystem.hpp"

using namespace aruwsrc::agitator;
using namespace aruwsrc::control;
using namespace aruwlib::algorithms;
using namespace aruwlib::sensors;
using namespace aruwlib;
using namespace aruwsrc::chassis;

/* define subsystems --------------------------------------------------------*/
#if defined(TARGET_SOLDIER)
AgitatorSubsystem agitator17mm(AgitatorSubsystem::AgitatorType::Soldier);
ExampleSubsystem frictionWheelSubsystem;
#endif

/* define commands ----------------------------------------------------------*/
#if defined(TARGET_SOLDIER)
aruwsrc::control::ExampleCommand spinFrictionWheelCommand(&frictionWheelSubsystem,
    ExampleCommand::DEFAULT_WHEEL_RPM);
ShootSlowComprisedCommand agitatorShootSlowCommand(&agitator17mm);
AgitatorCalibrateCommand agitatorCalibrateCommand(&agitator17mm);
#endif

using namespace aruwlib::sensors;

#if defined(TARGET_SOLDIER)
ChassisSubsystem soldierChassis;
ChassisDriveCommand chassisDriveCommand(&soldierChassis);
#else  // error
#error "select soldier robot type only"
#endif

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

    aruwlib::serial::RefSerial::getRefSerial().initialize();
    aruwlib::serial::XavierSerial::getXavierSerial().initialize();

    /* register subsystems here ---------------------------------------------*/
    #if defined(TARGET_SOLDIER)
    CommandScheduler::getMainScheduler().registerSubsystem(&agitator17mm);
    CommandScheduler::getMainScheduler().registerSubsystem(&frictionWheelSubsystem);
    #endif

    /* set any default commands to subsystems here --------------------------*/
    #if defined(TARGET_SOLDIER)
    frictionWheelSubsystem.setDefaultCommand(&spinFrictionWheelCommand);
    #endif

    /* add any starting commands to the scheduler here ----------------------*/
    #if defined(TARGET_SOLDIER)
    CommandScheduler::getMainScheduler().addCommand(&agitatorCalibrateCommand);
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

    while (1)
    {
        // do this as fast as you can
        aruwlib::can::CanRxHandler::pollCanData();
        aruwlib::serial::XavierSerial::getXavierSerial().updateSerial();
        aruwlib::serial::RefSerial::getRefSerial().updateSerial();

        Remote::read();

        if (updateImuPeriod.execute())
        {
            Mpu6500::read();
        }
        
        if (sendMotorTimeout.execute())
        {
            CommandScheduler::getMainScheduler().run();
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }

        modm::delayMicroseconds(10);
    }
    return 0;
}
