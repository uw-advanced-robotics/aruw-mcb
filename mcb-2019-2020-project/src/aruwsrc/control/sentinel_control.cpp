#include <aruwlib/Drivers.hpp>
#include <aruwlib/control/command_mapper.hpp>

#include "agitator/agitator_calibrate_command.hpp"
#include "agitator/agitator_shoot_comprised_command_instances.hpp"
#include "agitator/agitator_subsystem.hpp"
#include "launcher/friction_wheel_rotate_command.hpp"
#include "launcher/friction_wheel_subsystem.hpp"
#include "sentinel/sentinel_auto_drive_command.hpp"
#include "sentinel/sentinel_drive_manual_command.hpp"
#include "sentinel/sentinel_drive_subsystem.hpp"
#include "turret/turret_cv_command.hpp"
#include "turret/turret_init_command.hpp"
#include "turret/turret_manual_command.hpp"
#include "turret/turret_subsystem.hpp"
#include "turret/turret_world_relative_position_command.hpp"

#if defined(TARGET_SENTINEL)

using namespace aruwsrc::agitator;
using namespace aruwsrc::launcher;
using namespace aruwsrc::control;
using aruwlib::Drivers;
using aruwlib::control::CommandMapper;

namespace aruwsrc
{
namespace control
{
/* define subsystems --------------------------------------------------------*/
AgitatorSubsystem<Drivers> agitator(
    AgitatorSubsystem<Drivers>::PID_17MM_P,
    AgitatorSubsystem<Drivers>::PID_17MM_I,
    AgitatorSubsystem<Drivers>::PID_17MM_D,
    AgitatorSubsystem<Drivers>::PID_17MM_MAX_ERR_SUM,
    AgitatorSubsystem<Drivers>::PID_17MM_MAX_OUT,
    AgitatorSubsystem<Drivers>::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem<Drivers>::AGITATOR_MOTOR_ID,
    AgitatorSubsystem<Drivers>::AGITATOR_MOTOR_CAN_BUS,
    false);

AgitatorSubsystem<Drivers> kickerMotor(
    AgitatorSubsystem<Drivers>::PID_17MM_KICKER_P,
    AgitatorSubsystem<Drivers>::PID_17MM_KICKER_I,
    AgitatorSubsystem<Drivers>::PID_17MM_KICKER_D,
    AgitatorSubsystem<Drivers>::PID_17MM_KICKER_MAX_ERR_SUM,
    AgitatorSubsystem<Drivers>::PID_17MM_KICKER_MAX_OUT,
    AgitatorSubsystem<Drivers>::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem<Drivers>::SENTINEL_KICKER_MOTOR_ID,
    AgitatorSubsystem<Drivers>::AGITATOR_MOTOR_CAN_BUS,
    false);

SentinelDriveSubsystem<Drivers> sentinelDrive;

FrictionWheelSubsystem<Drivers> upperFrictionWheels(aruwlib::motor::MOTOR3, aruwlib::motor::MOTOR4);

FrictionWheelSubsystem<Drivers> lowerFrictionWheels;

/* define commands ----------------------------------------------------------*/
ShootFastComprisedCommand<Drivers> agitatorShootSlowCommand(&agitator);

AgitatorCalibrateCommand<Drivers> agitatorCalibrateCommand(&agitator);

AgitatorRotateCommand<Drivers> agitatorKickerCommand(&kickerMotor, 3.0f, 1, 0, false);

AgitatorCalibrateCommand<Drivers> agitatorCalibrateKickerCommand(&kickerMotor);

SentinelAutoDriveCommand<Drivers> sentinelAutoDrive(&sentinelDrive);

SentinelDriveManualCommand<Drivers> sentinelDriveManual(&sentinelDrive);

FrictionWheelRotateCommand<Drivers> spinUpperFrictionWheels(
    &upperFrictionWheels,
    FrictionWheelRotateCommand<Drivers>::DEFAULT_WHEEL_RPM);

FrictionWheelRotateCommand<Drivers> spinLowerFrictionWheels(
    &lowerFrictionWheels,
    FrictionWheelRotateCommand<Drivers>::DEFAULT_WHEEL_RPM);

FrictionWheelRotateCommand<Drivers> stopUpperFrictionWheels(&upperFrictionWheels, 0);

FrictionWheelRotateCommand<Drivers> stopLowerFrictionWheels(&lowerFrictionWheels, 0);

/* register subsystems here -------------------------------------------------*/
void registerSentinelSubsystems()
{
    Drivers::commandScheduler.registerSubsystem(&agitator);
    Drivers::commandScheduler.registerSubsystem(&kickerMotor);
    Drivers::commandScheduler.registerSubsystem(&sentinelDrive);
    Drivers::commandScheduler.registerSubsystem(&upperFrictionWheels);
    Drivers::commandScheduler.registerSubsystem(&lowerFrictionWheels);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSentinelCommands()
{
    sentinelDrive.setDefaultCommand(&sentinelDriveManual);
    upperFrictionWheels.setDefaultCommand(&spinUpperFrictionWheels);
    lowerFrictionWheels.setDefaultCommand(&spinLowerFrictionWheels);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSentinelCommands()
{
    Drivers::commandScheduler.addCommand(&agitatorCalibrateCommand);
    Drivers::commandScheduler.addCommand(&agitatorCalibrateKickerCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerSentinelIoMappings()
{
    Drivers::commandMapper.addHoldRepeatMapping(
        CommandMapper::newKeyMap(
            aruwlib::Remote::Switch::LEFT_SWITCH,
            aruwlib::Remote::SwitchState::UP),
        &agitatorShootSlowCommand);

    Drivers::commandMapper.addHoldRepeatMapping(
        CommandMapper::newKeyMap(
            aruwlib::Remote::Switch::RIGHT_SWITCH,
            aruwlib::Remote::SwitchState::UP),
        &agitatorKickerCommand);

    Drivers::commandMapper.addHoldRepeatMapping(
        CommandMapper::newKeyMap(
            aruwlib::Remote::Switch::RIGHT_SWITCH,
            aruwlib::Remote::SwitchState::DOWN),
        &sentinelAutoDrive);

    Drivers::commandMapper.addHoldMapping(
        CommandMapper::newKeyMap(
            aruwlib::Remote::Switch::LEFT_SWITCH,
            aruwlib::Remote::SwitchState::DOWN),
        &stopLowerFrictionWheels);

    Drivers::commandMapper.addHoldMapping(
        CommandMapper::newKeyMap(
            aruwlib::Remote::Switch::RIGHT_SWITCH,
            aruwlib::Remote::SwitchState::DOWN),
        &stopUpperFrictionWheels);
}

void initSubsystemCommands()
{
    registerSentinelSubsystems();
    setDefaultSentinelCommands();
    startSentinelCommands();
    registerSentinelIoMappings();
}

}  // namespace control

}  // namespace aruwsrc

#endif
