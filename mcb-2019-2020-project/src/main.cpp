#include <rm-dev-board-a/board.hpp>
#include <modm/processing/timer.hpp>

/* communication includes ---------------------------------------------------*/
#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "src/aruwlib/communication/serial/xavier_serial.hpp"
#include "src/aruwlib/communication/serial/ref_serial.hpp"
#include "src/aruwlib/display/sh1106.hpp"

/* aruwlib control includes -------------------------------------------------*/
#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwlib/control/controller_mapper.hpp"

/* math includes ------------------------------------------------------------*/
#include "aruwlib/algorithms/contiguous_float_test.hpp"

/* aruwsrc control includes -------------------------------------------------*/
#include "src/aruwsrc/control/example/example_command.hpp"
#include "src/aruwsrc/control/example/example_comprised_command.hpp"
#include "src/aruwsrc/control/example/example_subsystem.hpp"
#include "src/aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "src/aruwsrc/control/agitator/agitator_calibrate_command.hpp"
#include "src/aruwsrc/control/agitator/agitator_shoot_comprised_command_instances.hpp"
#include "src/aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "src/aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "src/aruwsrc/control/turret/turret_subsystem.hpp"
#include "src/aruwsrc/control/turret/turret_cv_command.hpp"
#include "src/aruwsrc/control/turret/turret_init_command.hpp"
#include "src/aruwsrc/control/turret/turret_manual_command.hpp"
#include "src/aruwsrc/control/turret/turret_world_relative_position_command.hpp"
#include "src/aruwsrc/control/chassis/chassis_autorotate_command.hpp"

/* error handling includes --------------------------------------------------*/
#include "src/aruwlib/errors/error_controller.hpp"

using namespace aruwsrc::agitator;
using namespace aruwsrc::control;
using namespace aruwlib::sensors;
using namespace aruwlib;
using namespace aruwsrc::chassis;
using namespace aruwsrc::control;
using namespace aruwlib::sensors;

/* define subsystems --------------------------------------------------------*/
#if defined(TARGET_SOLDIER)
TurretSubsystem turretSubsystem;
TurretCVCommand turretCVCommand(&turretSubsystem);

ChassisSubsystem soldierChassis;

AgitatorSubsystem agitator17mm(
    AgitatorSubsystem::PID_17MM_P,
    AgitatorSubsystem::PID_17MM_I,
    AgitatorSubsystem::PID_17MM_D,
    AgitatorSubsystem::PID_17MM_MAX_ERR_SUM,
    AgitatorSubsystem::PID_17MM_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem::AGITATOR_MOTOR_ID,
    AgitatorSubsystem::AGITATOR_MOTOR_CAN_BUS,
    AgitatorSubsystem::isAgitatorInverted
);

ExampleSubsystem frictionWheelSubsystem;

#elif defined(TARGET_SENTRY)
AgitatorSubsystem sentryAgitator(
    AgitatorSubsystem::PID_17MM_P,
    AgitatorSubsystem::PID_17MM_I,
    AgitatorSubsystem::PID_17MM_D,
    AgitatorSubsystem::PID_17MM_MAX_ERR_SUM,
    AgitatorSubsystem::PID_17MM_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem::AGITATOR_MOTOR_ID,
    AgitatorSubsystem::AGITATOR_MOTOR_CAN_BUS,
    false
);

AgitatorSubsystem sentryKicker(
    AgitatorSubsystem::PID_17MM_KICKER_P,
    AgitatorSubsystem::PID_17MM_KICKER_I,
    AgitatorSubsystem::PID_17MM_KICKER_D,
    AgitatorSubsystem::PID_17MM_KICKER_MAX_ERR_SUM,
    AgitatorSubsystem::PID_17MM_KICKER_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem::SENTRY_KICKER_MOTOR_ID,
    AgitatorSubsystem::AGITATOR_MOTOR_CAN_BUS,
    false
);

ExampleSubsystem frictionWheelSubsystem;
#endif

/* define commands ----------------------------------------------------------*/

#if defined(TARGET_SOLDIER)
ChassisAutorotateCommand chassisAutorotateCommand(&soldierChassis, &turretSubsystem);
aruwsrc::control::ExampleCommand spinFrictionWheelCommand(&frictionWheelSubsystem,
        ExampleCommand::DEFAULT_WHEEL_RPM);

TurretWorldRelativePositionCommand turretUserCommand(&turretSubsystem, &soldierChassis);

ShootFastComprisedCommand agitatorShootSlowCommand(&agitator17mm);
AgitatorCalibrateCommand agitatorCalibrateCommand(&agitator17mm);
#elif defined(TARGET_SENTRY)
aruwsrc::control::ExampleCommand spinFrictionWheelCommand(&frictionWheelSubsystem,
        ExampleCommand::DEFAULT_WHEEL_RPM);

ShootFastComprisedCommand agitatorShootSlowCommand(&sentryAgitator);
AgitatorCalibrateCommand agitatorCalibrateCommand(&sentryAgitator);
AgitatorRotateCommand agitatorKickerCommand(&sentryKicker, 3.0f, 1, 0, false);
AgitatorCalibrateCommand agitatorCalibrateKickerCommand(&sentryKicker);
#endif


int main()
{
    Board::initialize();

    Board::DisplaySpiMaster::connect<
        Board::DisplayMiso::Miso,
        Board::DisplayMosi::Mosi,
        Board::DisplaySck::Sck
    >();

    // SPI1 is on ABP2 which is at 90MHz; use prescaler 64 to get ~fastest baud rate below 1mHz max
    // 90MHz/64=~14MHz
    Board::DisplaySpiMaster::initialize<Board::SystemClock, 1406250_Hz>();

    aruwlib::display::Sh1106<
        Board::DisplaySpiMaster,
        Board::DisplayCommand,
        Board::DisplayReset,
        128, 64,
        false
    > display;
    display.initializeBlocking();
    display.setCursor(2, 1);
    display.setFont(modm::font::ScriptoNarrow);
    display << "ur code is shit" << modm::endl;
    display.update();

    aruwlib::algorithms::ContiguousFloatTest contiguousFloatTest;
    contiguousFloatTest.testCore();
    contiguousFloatTest.testBadBounds();
    contiguousFloatTest.testDifference();
    contiguousFloatTest.testRotationBounds();
    contiguousFloatTest.testShiftingValue();
    contiguousFloatTest.testWrapping();

    aruwlib::Remote::initialize();
    aruwlib::sensors::Mpu6500::init();

    aruwlib::serial::RefSerial::getRefSerial().initialize();
    aruwlib::serial::XavierSerial::getXavierSerial().initialize();

    /* register subsystems here ---------------------------------------------*/
    #if defined(TARGET_SOLDIER)
    CommandScheduler::getMainScheduler().registerSubsystem(&agitator17mm);
    CommandScheduler::getMainScheduler().registerSubsystem(&frictionWheelSubsystem);
    CommandScheduler::getMainScheduler().registerSubsystem(&soldierChassis);
    CommandScheduler::getMainScheduler().registerSubsystem(&turretSubsystem);
    #elif defined(TARGET_SENTRY)
    CommandScheduler::getMainScheduler().registerSubsystem(&sentryAgitator);
    CommandScheduler::getMainScheduler().registerSubsystem(&sentryKicker);
    CommandScheduler::getMainScheduler().registerSubsystem(&frictionWheelSubsystem);
    #endif

    /* set any default commands to subsystems here --------------------------*/
    #if defined(TARGET_SOLDIER)
    soldierChassis.setDefaultCommand(&chassisAutorotateCommand);
    turretSubsystem.setDefaultCommand(&turretUserCommand);
    frictionWheelSubsystem.setDefaultCommand(&spinFrictionWheelCommand);
    #elif defined(TARGET_SENTRY)
    frictionWheelSubsystem.setDefaultCommand(&spinFrictionWheelCommand);
    #endif

    /* add any starting commands to the scheduler here ----------------------*/
    #if defined(TARGET_SOLDIER)
    CommandScheduler::getMainScheduler().addCommand(&agitatorCalibrateCommand);
    #elif defined(TARGET_SENTRY)
    CommandScheduler::getMainScheduler().addCommand(&agitatorCalibrateCommand);
    CommandScheduler::getMainScheduler().addCommand(&agitatorCalibrateKickerCommand);
    #endif

    /* register io mappings here --------------------------------------------*/
    #if defined(TARGET_SOLDIER)
    IoMapper::addHoldRepeatMapping(
        IoMapper::newKeyMap(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
        &agitatorShootSlowCommand
    );
    #elif defined(TARGET_SENTRY)
    IoMapper::addHoldRepeatMapping(
        IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP),
        &agitatorShootSlowCommand
    );
    IoMapper::addHoldRepeatMapping(
        IoMapper::newKeyMap(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
        &agitatorKickerCommand
    );
    #endif

    /* define timers here ---------------------------------------------------*/
    modm::ShortPeriodicTimer updateImuPeriod(2);
    modm::ShortPeriodicTimer sendMotorTimeout(2);

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
            aruwlib::errors::ErrorController::update();
            CommandScheduler::getMainScheduler().run();
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }

        modm::delayMicroseconds(10);
    }
    return 0;
}
