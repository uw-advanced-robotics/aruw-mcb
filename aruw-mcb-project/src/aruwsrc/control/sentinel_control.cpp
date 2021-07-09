/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#if defined(TARGET_SENTINEL)

#include "aruwlib/control/command_mapper.hpp"
#include "aruwlib/control/hold_command_mapping.hpp"
#include "aruwlib/control/hold_repeat_command_mapping.hpp"
#include "aruwlib/control/press_command_mapping.hpp"
#include "aruwlib/control/setpoint/commands/calibrate_command.hpp"
#include "aruwlib/control/toggle_command_mapping.hpp"
#include "aruwlib/control/turret/commands/turret_setpoint_command.hpp"
#include "aruwlib/drivers_singleton.hpp"

#include "agitator/agitator_shoot_comprised_command_instances.hpp"
#include "agitator/agitator_subsystem.hpp"
#include "constants/robot_constants.hpp"
#include "launcher/friction_wheel_rotate_command.hpp"
#include "launcher/friction_wheel_subsystem.hpp"
#include "sentinel/drive/sentinel_auto_drive_comprised_command.hpp"
#include "sentinel/drive/sentinel_drive_manual_command.hpp"
#include "sentinel/drive/sentinel_drive_subsystem.hpp"
#include "sentinel/firing/sentinel_rotate_agitator_command.hpp"
#include "sentinel/firing/sentinel_switcher_subsystem.hpp"
#include "turret/double_pitch_turret_subsystem.hpp"
#include "turret/sentinel_turret_cv_command.hpp"

using namespace aruwlib::control::setpoint;
using namespace aruwsrc::agitator;
using namespace aruwsrc::launcher;
using namespace aruwsrc::control::sentinel::firing;
using namespace aruwsrc::control::sentinel::drive;
using namespace aruwsrc::control;
using namespace aruwlib::control;
using namespace aruwlib::motor;
using aruwlib::DoNotUse_getDrivers;
using aruwlib::Remote;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
aruwlib::driversFunc drivers = aruwlib::DoNotUse_getDrivers;

namespace sentinel_control
{
/* define subsystems --------------------------------------------------------*/
AgitatorSubsystem agitator(
    drivers(),
    constants::agitator::AGITATOR_PID_CONFIG,
    constants::agitator::AGITATOR_GEAR_RATIO,
    constants::motor::AGITATOR_MOTOR_ID,
    constants::can::AGITATOR_MOTOR_CAN_BUS,
    constants::agitator::AGITATOR_MOTOR_INVERTED,
    true,
    constants::agitator::JAMMING_DISTANCE,
    constants::agitator::JAMMING_TIME);

SentinelDriveSubsystem sentinelDrive(
    drivers(),
    constants::chassis::CHASSIS_PID_CONFIG,
    constants::chassis::CHASSIS_POWER_LIMIT_CONFIG,
    constants::gpio::LEFT_LIMIT_SWITCH,
    constants::gpio::RIGHT_LIMIT_SWITCH,
    constants::gpio::CURRENT_SENSOR_PIN,
    constants::chassis::WHEEL_RADIUS,
    constants::chassis::GEAR_RATIO,
    constants::chassis::RAIL_LENGTH,
    constants::chassis::SENTINEL_LENGTH,
    constants::motor::CHASSIS_LEFT_MOTOR_ID,
    constants::motor::CHASSIS_RIGHT_MOTOR_ID,
    constants::can::CHASSIS_CAN_BUS);

FrictionWheelSubsystem upperFrictionWheels(
    drivers(),
    constants::launcher::LAUNCHER_PID_CONFIG,
    constants::launcher::FRICTION_WHEEL_RAMP_SPEED,
    constants::motor::LAUNCHER_LEFT_MOTOR_ID_UPPER,
    constants::motor::LAUNCHER_RIGHT_MOTOR_ID_UPPER,
    constants::can::LAUNCHER_CAN_BUS);

FrictionWheelSubsystem lowerFrictionWheels(
    drivers(),
    constants::launcher::LAUNCHER_PID_CONFIG,
    constants::launcher::FRICTION_WHEEL_RAMP_SPEED,
    constants::motor::LAUNCHER_LEFT_MOTOR_ID_LOWER,
    constants::motor::LAUNCHER_RIGHT_MOTOR_ID_LOWER,
    constants::can::LAUNCHER_CAN_BUS);

SentinelSwitcherSubsystem switcher(
    drivers(),
    constants::gpio::SWITCHER_SERVO_PIN,
    constants::launcher::SWITCHER_LOWER_PWM,
    constants::launcher::SWITCHER_UPPER_PWM);

aruwsrc::control::turret::DoublePitchTurretSubsystem turretSubsystem(
    drivers(),
    constants::turret::YAW_PID_CONFIG,
    constants::turret::PITCH_PID_CONFIG,
    constants::turret::USER_YAW_INPUT_SCALAR,
    constants::turret::USER_PITCH_INPUT_SCALAR,
    constants::turret::PITCH_GRAVITY_COMPENSATION_KP,
    constants::turret::TURRET_YAW_START_ANGLE,
    constants::turret::TURRET_PITCH_START_ANGLE,
    constants::turret::TURRET_YAW_MIN_ANGLE,
    constants::turret::TURRET_YAW_MAX_ANGLE,
    constants::turret::TURRET_PITCH_MIN_ANGLE,
    constants::turret::TURRET_PITCH_MAX_ANGLE,
    constants::turret::YAW_START_ENCODER_POSITION,
    constants::turret::PITCH_90DEG_ENCODER_POSITION_LEFT,
    constants::turret::PITCH_90DEG_ENCODER_POSITION_RIGHT,
    constants::can::TURRET_CAN_BUS,
    constants::motor::PITCH_MOTOR_ID_RIGHT,
    constants::motor::PITCH_MOTOR_ID_LEFT,
    constants::motor::YAW_MOTOR_ID);

/* define commands ----------------------------------------------------------*/
SentinelRotateAgitatorCommand rotateAgitatorManual(drivers(), &agitator, &switcher);

CalibrateCommand agitatorCalibrateCommand(&agitator);

// Two identical drive commands since you can't map an identical command to two different mappings
SentinelDriveManualCommand sentinelDriveManual(drivers(), &sentinelDrive);
SentinelDriveManualCommand sentinelDriveManual2(drivers(), &sentinelDrive);

FrictionWheelRotateCommand spinUpperFrictionWheels(
    &upperFrictionWheels,
    constants::launcher::FRICTION_WHEEL_TARGET_RPM);

FrictionWheelRotateCommand spinLowerFrictionWheels(
    &lowerFrictionWheels,
    constants::launcher::FRICTION_WHEEL_TARGET_RPM);

FrictionWheelRotateCommand stopUpperFrictionWheels(&upperFrictionWheels, 0);

FrictionWheelRotateCommand stopLowerFrictionWheels(&lowerFrictionWheels, 0);

aruwsrc::control::turret::SentinelTurretCVCommand turretCVCommand(
    drivers(),
    &turretSubsystem,
    &agitator,
    &switcher,
    constants::turret::TURRET_YAW_MIN_ANGLE,
    constants::turret::TURRET_YAW_MAX_ANGLE,
    constants::turret::TURRET_PITCH_MIN_ANGLE,
    constants::turret::TURRET_PITCH_MAX_ANGLE);

aruwlib::control::turret::commands::TurretSetpointCommand turretManual(
    drivers(),
    &turretSubsystem,
    0.75f,
    0.75f);

SentinelAutoDriveComprisedCommand sentinelAutoDrive(drivers(), &sentinelDrive);

/* define command mappings --------------------------------------------------*/

HoldCommandMapping rightSwitchDown(
    drivers(),
    {&stopLowerFrictionWheels, &stopUpperFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&rotateAgitatorManual},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));
HoldCommandMapping leftSwitchDown(
    drivers(),
    {&sentinelDriveManual, &turretManual},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
HoldCommandMapping leftSwitchMid(
    drivers(),
    {&sentinelDriveManual2},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    agitator.initialize();
    sentinelDrive.initialize();
    upperFrictionWheels.initialize();
    lowerFrictionWheels.initialize();
    switcher.initialize();
    turretSubsystem.initialize();
    drivers()->xavierSerial.attachChassis(&sentinelDrive);
    drivers()->xavierSerial.attachTurret(&turretSubsystem);
}

/* register subsystems here -------------------------------------------------*/
void registerSentinelSubsystems(aruwlib::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&sentinelDrive);
    drivers->commandScheduler.registerSubsystem(&upperFrictionWheels);
    drivers->commandScheduler.registerSubsystem(&lowerFrictionWheels);
    drivers->commandScheduler.registerSubsystem(&switcher);
    drivers->commandScheduler.registerSubsystem(&turretSubsystem);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSentinelCommands(aruwlib::Drivers *)
{
    sentinelDrive.setDefaultCommand(&sentinelAutoDrive);
    upperFrictionWheels.setDefaultCommand(&spinUpperFrictionWheels);
    lowerFrictionWheels.setDefaultCommand(&spinLowerFrictionWheels);
    turretSubsystem.setDefaultCommand(&turretCVCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSentinelCommands(aruwlib::Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&agitatorCalibrateCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerSentinelIoMappings(aruwlib::Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchMid);
}
}  // namespace sentinel_control

namespace aruwsrc::control
{
void initSubsystemCommands(aruwlib::Drivers *drivers)
{
    sentinel_control::initializeSubsystems();
    sentinel_control::registerSentinelSubsystems(drivers);
    sentinel_control::setDefaultSentinelCommands(drivers);
    sentinel_control::startSentinelCommands(drivers);
    sentinel_control::registerSentinelIoMappings(drivers);
}  // namespace aruwsrc
}  // namespace aruwsrc::control

#endif
