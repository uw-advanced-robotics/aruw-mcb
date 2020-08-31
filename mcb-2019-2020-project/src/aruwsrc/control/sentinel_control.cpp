#include <aruwlib/HALDrivers.hpp>
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
using namespace aruwlib::remote;
using aruwlib::HALDrivers;
using aruwlib::control::CommandMapper;

namespace aruwsrc
{
namespace control
{
/* define subsystems --------------------------------------------------------*/
AgitatorSubsystem<HALDrivers> agitator(
    AgitatorSubsystem<HALDrivers>::PID_17MM_P,
    AgitatorSubsystem<HALDrivers>::PID_17MM_I,
    AgitatorSubsystem<HALDrivers>::PID_17MM_D,
    AgitatorSubsystem<HALDrivers>::PID_17MM_MAX_ERR_SUM,
    AgitatorSubsystem<HALDrivers>::PID_17MM_MAX_OUT,
    AgitatorSubsystem<HALDrivers>::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem<HALDrivers>::AGITATOR_MOTOR_ID,
    AgitatorSubsystem<HALDrivers>::AGITATOR_MOTOR_CAN_BUS,
    false);

AgitatorSubsystem<HALDrivers> kickerMotor(
    AgitatorSubsystem<HALDrivers>::PID_17MM_KICKER_P,
    AgitatorSubsystem<HALDrivers>::PID_17MM_KICKER_I,
    AgitatorSubsystem<HALDrivers>::PID_17MM_KICKER_D,
    AgitatorSubsystem<HALDrivers>::PID_17MM_KICKER_MAX_ERR_SUM,
    AgitatorSubsystem<HALDrivers>::PID_17MM_KICKER_MAX_OUT,
    AgitatorSubsystem<HALDrivers>::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem<HALDrivers>::SENTINEL_KICKER_MOTOR_ID,
    AgitatorSubsystem<HALDrivers>::AGITATOR_MOTOR_CAN_BUS,
    false);

SentinelDriveSubsystem<HALDrivers> sentinelDrive;

FrictionWheelSubsystem<HALDrivers> upperFrictionWheels(
    aruwlib::motor::MOTOR3,
    aruwlib::motor::MOTOR4);

FrictionWheelSubsystem<HALDrivers> lowerFrictionWheels;

/* define commands ----------------------------------------------------------*/
ShootFastComprisedCommand<HALDrivers> agitatorShootSlowCommand(&agitator);

AgitatorCalibrateCommand<HALDrivers> agitatorCalibrateCommand(&agitator);

AgitatorRotateCommand<HALDrivers> agitatorKickerCommand(&kickerMotor, 3.0f, 1, 0, false);

AgitatorCalibrateCommand<HALDrivers> agitatorCalibrateKickerCommand(&kickerMotor);

SentinelAutoDriveCommand<HALDrivers> sentinelAutoDrive(&sentinelDrive);

SentinelDriveManualCommand<HALDrivers> sentinelDriveManual(&sentinelDrive);

FrictionWheelRotateCommand<HALDrivers> spinUpperFrictionWheels(
    &upperFrictionWheels,
    FrictionWheelRotateCommand<HALDrivers>::DEFAULT_WHEEL_RPM);

FrictionWheelRotateCommand<HALDrivers> spinLowerFrictionWheels(
    &lowerFrictionWheels,
    FrictionWheelRotateCommand<HALDrivers>::DEFAULT_WHEEL_RPM);

FrictionWheelRotateCommand<HALDrivers> stopUpperFrictionWheels(&upperFrictionWheels, 0);

FrictionWheelRotateCommand<HALDrivers> stopLowerFrictionWheels(&lowerFrictionWheels, 0);

/* register subsystems here -------------------------------------------------*/
void registerSentinelSubsystems()
{
    HALDrivers::commandScheduler.registerSubsystem(&agitator);
    HALDrivers::commandScheduler.registerSubsystem(&kickerMotor);
    HALDrivers::commandScheduler.registerSubsystem(&sentinelDrive);
    HALDrivers::commandScheduler.registerSubsystem(&upperFrictionWheels);
    HALDrivers::commandScheduler.registerSubsystem(&lowerFrictionWheels);
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
    HALDrivers::commandScheduler.addCommand(&agitatorCalibrateCommand);
    HALDrivers::commandScheduler.addCommand(&agitatorCalibrateKickerCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerSentinelIoMappings()
{
    HALDrivers::commandMapper.addHoldRepeatMapping(
        CommandMapper<HALDrivers>::newKeyMap(Switch::LEFT_SWITCH, SwitchState::UP),
        &agitatorShootSlowCommand);

    HALDrivers::commandMapper.addHoldRepeatMapping(
        CommandMapper<HALDrivers>::newKeyMap(Switch::RIGHT_SWITCH, SwitchState::UP),
        &agitatorKickerCommand);

    HALDrivers::commandMapper.addHoldRepeatMapping(
        CommandMapper<HALDrivers>::newKeyMap(Switch::RIGHT_SWITCH, SwitchState::DOWN),
        &sentinelAutoDrive);

    HALDrivers::commandMapper.addHoldMapping(
        CommandMapper<HALDrivers>::newKeyMap(Switch::LEFT_SWITCH, SwitchState::DOWN),
        &stopLowerFrictionWheels);

    HALDrivers::commandMapper.addHoldMapping(
        CommandMapper<HALDrivers>::newKeyMap(Switch::RIGHT_SWITCH, SwitchState::DOWN),
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
