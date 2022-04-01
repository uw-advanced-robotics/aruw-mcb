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

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/motor/double_dji_motor.hpp"

#include "agitator/agitator_subsystem.hpp"
#include "agitator/constants/agitator_constants.hpp"
#include "agitator/move_unjam_ref_limited_command.hpp"
#include "aruwsrc/constants.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "launcher/friction_wheel_subsystem.hpp"
#include "sentinel/drive/sentinel_auto_drive_comprised_command.hpp"
#include "sentinel/drive/sentinel_drive_manual_command.hpp"
#include "sentinel/drive/sentinel_drive_subsystem.hpp"
#include "turret/algorithms/chassis_frame_turret_controller.hpp"
#include "turret/turret_controller_constants.hpp"
#include "turret/turret_subsystem.hpp"
#include "turret/user/turret_user_control_command.hpp"

using namespace tap::control::setpoint;
using namespace aruwsrc::agitator;
using namespace aruwsrc::control::sentinel::drive;
using namespace tap::gpio;
using namespace aruwsrc::control;
using namespace tap::control;
using namespace tap::motor;
using namespace aruwsrc::control::turret;
using namespace tap::communication::serial;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
aruwsrc::driversFunc drivers = aruwsrc::DoNotUse_getDrivers;

namespace sentinel_control
{
static constexpr Digital::InputPin LEFT_LIMIT_SWITCH = Digital::InputPin::A;
static constexpr Digital::InputPin RIGHT_LIMIT_SWITCH = Digital::InputPin::B;

/* define subsystems --------------------------------------------------------*/
AgitatorSubsystem agitator(
    drivers(),
    aruwsrc::control::agitator::constants::AGITATOR_PID_CONFIG,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    aruwsrc::control::agitator::constants::AGITATOR_MOTOR_ID,
    aruwsrc::control::agitator::constants::AGITATOR_MOTOR_CAN_BUS,
    false,
    M_PI / 10,
    150,
    true);

SentinelDriveSubsystem sentinelDrive(drivers(), LEFT_LIMIT_SWITCH, RIGHT_LIMIT_SWITCH);

aruwsrc::control::launcher::FrictionWheelSubsystem frictionWheels(drivers());

// Note: motor "one" is right, "two" is left
tap::motor::DjiMotor pitchMotor(
    drivers(),
    tap::motor::MOTOR5,
    CAN_BUS_MOTORS,
    false,
    "Pitch Turret");
tap::motor::DjiMotor yawMotor(drivers(), tap::motor::MOTOR6, CAN_BUS_MOTORS, true, "Yaw Turret");
TurretSubsystem turretSubsystem(drivers(), &pitchMotor, &yawMotor);

/* define commands ----------------------------------------------------------*/
aruwsrc::agitator::MoveUnjamRefLimitedCommand rotateAgitatorManual(
    drivers(),
    &agitator,
    M_PI / 5.0f,
    50,
    0,
    true,
    M_PI / 16.0f,
    M_PI / 2.0f,
    M_PI / 4.0f,
    130,
    2,
    true,
    10);

CalibrateCommand agitatorCalibrateCommand(&agitator);

// Two identical drive commands since you can't map an identical command to two different mappings
SentinelDriveManualCommand sentinelDriveManual(drivers(), &sentinelDrive);
SentinelDriveManualCommand sentinelDriveManual2(drivers(), &sentinelDrive);

aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand spinFrictionWheels(
    drivers(),
    &frictionWheels,
    30.0f,
    true,
    aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand stopFrictionWheels(
    drivers(),
    &frictionWheels,
    0.0f,
    true,
    aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

// turret controllers
algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController(
    &turretSubsystem,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
    &turretSubsystem,
    chassis_rel::YAW_PID_CONFIG);

// turret commands

user::TurretUserControlCommand turretManual(
    drivers(),
    &turretSubsystem,
    &chassisFrameYawTurretController,
    &chassisFramePitchTurretController);

SentinelAutoDriveComprisedCommand sentinelAutoDrive(drivers(), &sentinelDrive);

/* define command mappings --------------------------------------------------*/

HoldCommandMapping rightSwitchDown(
    drivers(),
    {&stopFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&rotateAgitatorManual},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    true);
HoldRepeatCommandMapping leftSwitchDown(
    drivers(),
    {&sentinelDriveManual, &turretManual},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN),
    true);
HoldCommandMapping leftSwitchMid(
    drivers(),
    {&sentinelDriveManual2},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    agitator.initialize();
    sentinelDrive.initialize();
    frictionWheels.initialize();
    turretSubsystem.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerSentinelSubsystems(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&sentinelDrive);
    drivers->commandScheduler.registerSubsystem(&frictionWheels);
    drivers->commandScheduler.registerSubsystem(&turretSubsystem);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSentinelCommands(aruwsrc::Drivers *)
{
    sentinelDrive.setDefaultCommand(&sentinelAutoDrive);
    frictionWheels.setDefaultCommand(&spinFrictionWheels);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSentinelCommands(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&agitatorCalibrateCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerSentinelIoMappings(aruwsrc::Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchMid);
}
}  // namespace sentinel_control

namespace aruwsrc::control
{
void initSubsystemCommands(aruwsrc::Drivers *drivers)
{
    sentinel_control::initializeSubsystems();
    sentinel_control::registerSentinelSubsystems(drivers);
    sentinel_control::setDefaultSentinelCommands(drivers);
    sentinel_control::startSentinelCommands(drivers);
    sentinel_control::registerSentinelIoMappings(drivers);
}  // namespace aruwsrc
}  // namespace aruwsrc::control

#endif
