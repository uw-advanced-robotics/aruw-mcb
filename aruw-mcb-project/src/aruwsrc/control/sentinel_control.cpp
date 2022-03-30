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
#include "agitator/move_unjam_ref_limited_command.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "launcher/friction_wheel_subsystem.hpp"
#include "sentinel/drive/sentinel_auto_drive_comprised_command.hpp"
#include "sentinel/drive/sentinel_drive_manual_command.hpp"
#include "sentinel/drive/sentinel_drive_subsystem.hpp"
#include "turret/algorithms/chassis_frame_turret_controller.hpp"
#include "turret/cv/sentinel_turret_cv_command.hpp"
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
using namespace aruwsrc::control::launcher;
using tap::Remote;

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
AgitatorSubsystem agitator1(
    drivers(),
    AgitatorSubsystem::PID_17MM_P,
    AgitatorSubsystem::PID_17MM_I,
    AgitatorSubsystem::PID_17MM_D,
    AgitatorSubsystem::PID_17MM_MAX_ERR_SUM,
    AgitatorSubsystem::PID_17MM_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem::AGITATOR_MOTOR_ID, // hard code
    AgitatorSubsystem::AGITATOR_MOTOR_CAN_BUS,
    false,
    M_PI / 10,
    150,
    true);
AgitatorSubsystem agitator2(
    drivers(),
    AgitatorSubsystem::PID_17MM_P,
    AgitatorSubsystem::PID_17MM_I,
    AgitatorSubsystem::PID_17MM_D,
    AgitatorSubsystem::PID_17MM_MAX_ERR_SUM,
    AgitatorSubsystem::PID_17MM_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem::AGITATOR_MOTOR_ID,
    AgitatorSubsystem::AGITATOR_MOTOR_CAN_BUS,
    false,
    M_PI / 10,
    150,
    true);

SentinelDriveSubsystem sentinelDrive(drivers(), LEFT_LIMIT_SWITCH, RIGHT_LIMIT_SWITCH);

FrictionWheelSubsystem frictionWheels1(drivers()); // add new motor for diff subsystems
FrictionWheelSubsystem frictionWheels2(drivers());

// Note: motor "one" is right, "two" is left
tap::motor::DjiMotor pitchMotor1(
    drivers(),
    tap::motor::MOTOR5,
    TurretSubsystem::CAN_BUS_MOTORS,
    false,
    "Pitch Turret");
tap::motor::DjiMotor yawMotor(
    drivers(),
    tap::motor::MOTOR6,
    TurretSubsystem::CAN_BUS_MOTORS,
    true,
    "Yaw Turret");
TurretSubsystem turretSubsystem1(drivers(), &pitchMotor1, &yawMotor1);
tap::motor::DjiMotor pitchMotor2(
    drivers(),
    tap::motor::MOTOR7,
    TurretSubsystem::CAN_BUS_MOTORS,
    false,
    "Pitch Turret");
tap::motor::DjiMotor yawMotor2(
    drivers(),
    tap::motor::MOTOR8,
    TurretSubsystem::CAN_BUS_MOTORS,
    true,
    "Yaw Turret");
TurretSubsystem turretSubsystem2(drivers(), &pitchMotor2, &yawMotor2);

/* define commands ----------------------------------------------------------*/
aruwsrc::agitator::MoveUnjamRefLimitedCommand rotateAgitatorManual1(
    drivers(),
    &agitator1,
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
aruwsrc::agitator::MoveUnjamRefLimitedCommand rotateAgitatorManual2(
    drivers(),
    &agitator2,
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

CalibrateCommand agitatorCalibrateCommand(&agitator1);
CalibrateCommand agitatorCalibrateCommand(&agitator2);

// Two identical drive commands since you can't map an identical command to two different mappings
SentinelDriveManualCommand sentinelDriveManual(drivers(), &sentinelDrive);
SentinelDriveManualCommand sentinelDriveManual2(drivers(), &sentinelDrive);

FrictionWheelSpinRefLimitedCommand spinFrictionWheels1(
    drivers(),
    &frictionWheels1,
    30.0f,
    true,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

FrictionWheelSpinRefLimitedCommand stopFrictionWheels1(
    drivers(),
    &frictionWheels1,
    0.0f,
    true,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

FrictionWheelSpinRefLimitedCommand spinFrictionWheels2(
    drivers(),
    &frictionWheels2,
    30.0f,
    true,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

FrictionWheelSpinRefLimitedCommand stopFrictionWheels2(
    drivers(),
    &frictionWheels2,
    0.0f,
    true,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

// turret controllers
algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController1(
    &turretSubsystem1,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController1(
    &turretSubsystem1,
    chassis_rel::YAW_PID_CONFIG);

algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController2(
    &turretSubsystem2,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController2(
    &turretSubsystem2,
    chassis_rel::YAW_PID_CONFIG);

// turret commands
cv::SentinelTurretCVCommand turretCVCommand1(
    drivers(),
    &turretSubsystem1,
    &agitator1,
    &chassisFrameYawTurretController1,
    &chassisFramePitchTurretController1);

cv::SentinelTurretCVCommand turretCVCommand2(
    drivers(),
    &turretSubsystem2,
    &agitator2,
    &chassisFrameYawTurretController2,
    &chassisFramePitchTurretController2);

user::TurretUserControlCommand turretManual1(
    drivers(),
    &turretSubsystem1,
    &chassisFrameYawTurretController1,
    &chassisFramePitchTurretController1);

user::TurretUserControlCommand turretManual2(
    drivers(),
    &turretSubsystem2,
    &chassisFrameYawTurretController2,
    &chassisFramePitchTurretController2);

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
    agitator1.initialize();
    agitator2.initialize();
    sentinelDrive.initialize();
    frictionWheels1.initialize();
    frictionWheels2.initialize();
    turretSubsystem1.initialize();
    turretSubsystem2.initialize();
    drivers()->legacyVisionCoprocessor.attachChassis(&sentinelDrive);
    drivers()->legacyVisionCoprocessor.attachTurret(&turretSubsystem1);
    drivers()->legacyVisionCoprocessor.attachTurret(&turretSubsystem2);
}

/* register subsystems here -------------------------------------------------*/
void registerSentinelSubsystems(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&agitator1);
    drivers->commandScheduler.registerSubsystem(&agitator2);
    drivers->commandScheduler.registerSubsystem(&sentinelDrive);
    drivers->commandScheduler.registerSubsystem(&frictionWheels1);
    drivers->commandScheduler.registerSubsystem(&frictionWheels2);
    drivers->commandScheduler.registerSubsystem(&turretSubsystem1);
    drivers->commandScheduler.registerSubsystem(&turretSubsystem2);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSentinelCommands(aruwsrc::Drivers *)
{
    sentinelDrive.setDefaultCommand(&sentinelAutoDrive);
    frictionWheels.setDefaultCommand(&spinFrictionWheels1);
    frictionWheels.setDefaultCommand(&spinFrictionWheels2);
    turretSubsystem.setDefaultCommand(&turretCVCommand1);
    turretSubsystem.setDefaultCommand(&turretCVCommand2);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSentinelCommands(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&agitator1CalibrateCommand);
    drivers->commandScheduler.addCommand(&agitator2CalibrateCommand);
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
