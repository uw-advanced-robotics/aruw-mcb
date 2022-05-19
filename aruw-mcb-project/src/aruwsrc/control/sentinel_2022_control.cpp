/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#if defined(TARGET_SENTINEL_2022)

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/motor/double_dji_motor.hpp"

#include "agitator/agitator_subsystem.hpp"
#include "agitator/constants/agitator_constants.hpp"
#include "agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/otto_velocity_odometry_2d_subsystem.hpp"
#include "aruwsrc/communication/serial/sentinel_request_handler.hpp"
#include "aruwsrc/communication/serial/sentinel_request_message_types.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "sentinel/drive/sentinel_auto_drive_comprised_command.hpp"
#include "sentinel/drive/sentinel_drive_manual_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "sentinel/drive/sentinel_drive_subsystem.hpp"
#include "turret/algorithms/chassis_frame_turret_controller.hpp"
#include "turret/constants/turret_constants.hpp"
#include "governor/heat_limit_governor.hpp"
#include "turret/cv/sentinel_turret_cv_command.hpp"
#include "turret/sentinel_turret_subsystem.hpp"
#include "tap/control/conditionally_executed_command.hpp"
#include "turret/user/turret_user_control_command.hpp"

using namespace tap::control::setpoint;
using namespace tap::control::setpoint;
using namespace aruwsrc::agitator;
using namespace aruwsrc::control::sentinel::drive;
using namespace tap::gpio;
using namespace aruwsrc::control;
using namespace tap::control;
using namespace tap::motor;
using namespace aruwsrc::control::governor;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::control::agitator;
using namespace aruwsrc::control::launcher;
using namespace aruwsrc::algorithms::odometry;
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
static constexpr Digital::InputPin LEFT_LIMIT_SWITCH = Digital::InputPin::B;
static constexpr Digital::InputPin RIGHT_LIMIT_SWITCH = Digital::InputPin::C;

aruwsrc::communication::serial::SentinelRequestHandler sentinelRequestHandler(drivers());

/* define subsystems --------------------------------------------------------*/
SentinelDriveSubsystem sentinelDrive(drivers(), LEFT_LIMIT_SWITCH, RIGHT_LIMIT_SWITCH);

namespace turret0
{
VelocityAgitatorSubsystem agitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::turret0::AGITATOR_CONFIG);

aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    frictionWheels(
        drivers(),
        aruwsrc::control::launcher::LEFT_MOTOR_ID,
        aruwsrc::control::launcher::RIGHT_MOTOR_ID,
        aruwsrc::control::launcher::TURRET0_CAN_BUS_MOTORS,
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_2);

DjiMotor pitchMotor(
    drivers(),
    aruwsrc::control::turret::PITCH_MOTOR_ID,
    aruwsrc::control::turret::turret0::CAN_BUS_MOTORS,
    false,
    "Pitch Turret 0");
DjiMotor yawMotor(
    drivers(),
    aruwsrc::control::turret::YAW_MOTOR_ID,
    aruwsrc::control::turret::turret0::CAN_BUS_MOTORS,
    true,
    "Yaw Turret 0");
SentinelTurretSubsystem turretSubsystem(
    drivers(),
    &pitchMotor,
    &yawMotor,
    aruwsrc::control::turret::turret0::PITCH_MOTOR_CONFIG,
    aruwsrc::control::turret::turret0::YAW_MOTOR_CONFIG);
}  // namespace turret0

namespace turret1
{
VelocityAgitatorSubsystem agitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::turret1::AGITATOR_CONFIG);

aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    frictionWheels(
        drivers(),
        aruwsrc::control::launcher::LEFT_MOTOR_ID,
        aruwsrc::control::launcher::RIGHT_MOTOR_ID,
        aruwsrc::control::launcher::TURRET1_CAN_BUS_MOTORS,
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

DjiMotor pitchMotor(
    drivers(),
    aruwsrc::control::turret::PITCH_MOTOR_ID,
    aruwsrc::control::turret::turret1::CAN_BUS_MOTORS,
    true,
    "Pitch Turret 1");
DjiMotor yawMotor(
    drivers(),
    aruwsrc::control::turret::YAW_MOTOR_ID,
    aruwsrc::control::turret::turret1::CAN_BUS_MOTORS,
    true,
    "Yaw Turret 1");
SentinelTurretSubsystem turretSubsystem(
    drivers(),
    &pitchMotor,
    &yawMotor,
    aruwsrc::control::turret::turret1::PITCH_MOTOR_CONFIG,
    aruwsrc::control::turret::turret1::YAW_MOTOR_CONFIG);
}  // namespace turret1

OttoVelocityOdometry2DSubsystem odometrySubsystem(
    drivers(),
    &turret1::turretSubsystem.yawMotor,
    &sentinelDrive);

/* define commands ----------------------------------------------------------*/
// Two identical drive commands since you can't map an identical command to two different mappings
SentinelDriveManualCommand sentinelDriveManual1(drivers(), &sentinelDrive);
SentinelDriveManualCommand sentinelDriveManual2(drivers(), &sentinelDrive);

SentinelAutoDriveComprisedCommand sentinelAutoDrive(drivers(), &sentinelDrive);

namespace turret0
{
MoveIntegralCommand rotateAgitator(
    agitator,
    constants::AGITATOR_ROTATE_CONFIG);

UnjamIntegralCommand unjamAgitator(
    agitator,
    constants::AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(*drivers(), agitator, rotateAgitator, unjamAgitator);

// rotates agitator with heat limiting applied
HeatLimitGovernor heatLimitGovernor(
    *drivers(),
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_2,
    constants::HEAT_LIMIT_BUFFER);
ConditionallyExecutedCommand<1> rotateAndUnjamAgitatorWithHeatLimiting(
    {&agitator},
    rotateAndUnjamAgitator,
    {&heatLimitGovernor});

FrictionWheelSpinRefLimitedCommand spinFrictionWheels(
    drivers(),
    &frictionWheels,
    30.0f,
    true,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

FrictionWheelSpinRefLimitedCommand stopFrictionWheels(
    drivers(),
    &frictionWheels,
    0.0f,
    true,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

// turret controllers
algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController(
    &turretSubsystem.pitchMotor,
    chassis_rel::turret0::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
    &turretSubsystem.yawMotor,
    chassis_rel::turret0::YAW_PID_CONFIG);

// turret commands

user::TurretUserControlCommand turretManual(
    drivers(),
    &turretSubsystem,
    &chassisFrameYawTurretController,
    &chassisFramePitchTurretController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR,
    0);

cv::SentinelTurretCVCommand turretCVCommand(
    drivers(),
    &turretSubsystem,
    &chassisFrameYawTurretController,
    &chassisFramePitchTurretController,
    agitator,
    &rotateAndUnjamAgitatorWithHeatLimiting,
    odometrySubsystem,
    frictionWheels,
    29.5f,
    0);
}  // namespace turret0

namespace turret1
{
MoveIntegralCommand rotateAgitator(
    agitator,
    constants::AGITATOR_ROTATE_CONFIG);

UnjamIntegralCommand unjamAgitator(
    agitator,
    constants::AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(*drivers(), agitator, rotateAgitator, unjamAgitator);

// rotates agitator with heat limiting applied
HeatLimitGovernor heatLimitGovernor(
    *drivers(),
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1,
    constants::HEAT_LIMIT_BUFFER);
ConditionallyExecutedCommand<1> rotateAndUnjamAgitatorWithHeatLimiting(
    {&agitator},
    rotateAndUnjamAgitator,
    {&heatLimitGovernor});

FrictionWheelSpinRefLimitedCommand spinFrictionWheels(
    drivers(),
    &frictionWheels,
    30.0f,
    true,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

FrictionWheelSpinRefLimitedCommand stopFrictionWheels(
    drivers(),
    &frictionWheels,
    0.0f,
    true,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

// turret controllers
algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController(
    &turretSubsystem.pitchMotor,
    chassis_rel::turret1::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
    &turretSubsystem.yawMotor,
    chassis_rel::turret1::YAW_PID_CONFIG);

// turret commands

user::TurretUserControlCommand turretManual(
    drivers(),
    &turretSubsystem,
    &chassisFrameYawTurretController,
    &chassisFramePitchTurretController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR,
    1);

cv::SentinelTurretCVCommand turretCVCommand(
    drivers(),
    &turretSubsystem,
    &chassisFrameYawTurretController,
    &chassisFramePitchTurretController,
    agitator,
    &rotateAndUnjamAgitatorWithHeatLimiting,
    odometrySubsystem,
    frictionWheels,
    29.5f,
    1);
}  // namespace turret1

void selectNewRobotMessageHandler()
{
    turret0::turretCVCommand.requestNewTarget();
    turret1::turretCVCommand.requestNewTarget();
}
void targetNewQuadrantMessageHandler()
{
    turret0::turretCVCommand.changeScanningQuadrant();
    turret1::turretCVCommand.changeScanningQuadrant();
}

/* define command mappings --------------------------------------------------*/

HoldCommandMapping rightSwitchDown(
    drivers(),
    {&turret0::stopFrictionWheels, &turret1::stopFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&turret0::rotateAndUnjamAgitatorWithHeatLimiting, &turret1::rotateAndUnjamAgitatorWithHeatLimiting},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    true);
HoldRepeatCommandMapping leftSwitchDown(
    drivers(),
    {&sentinelDriveManual1, &turret0::turretManual, &turret1::turretManual},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN),
    true);
HoldCommandMapping leftSwitchMid(
    drivers(),
    {&sentinelDriveManual2},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    sentinelDrive.initialize();
    turret0::agitator.initialize();
    turret0::frictionWheels.initialize();
    turret0::turretSubsystem.initialize();
    turret1::agitator.initialize();
    turret1::frictionWheels.initialize();
    turret1::turretSubsystem.initialize();
    odometrySubsystem.initialize();
}

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* register subsystems here -------------------------------------------------*/
void registerSentinelSubsystems(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&sentinelDrive);
    drivers->commandScheduler.registerSubsystem(&turret0::agitator);
    drivers->commandScheduler.registerSubsystem(&turret0::frictionWheels);
    drivers->commandScheduler.registerSubsystem(&turret0::turretSubsystem);
    drivers->commandScheduler.registerSubsystem(&turret1::agitator);
    drivers->commandScheduler.registerSubsystem(&turret1::frictionWheels);
    drivers->commandScheduler.registerSubsystem(&turret1::turretSubsystem);
    drivers->commandScheduler.registerSubsystem(&odometrySubsystem);
    drivers->visionCoprocessor.attachOdometryInterface(&odometrySubsystem);
    drivers->visionCoprocessor.attachTurretOrientationInterface(&turret0::turretSubsystem, 1);
    drivers->visionCoprocessor.attachTurretOrientationInterface(&turret1::turretSubsystem, 0);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSentinelCommands(aruwsrc::Drivers *)
{
    sentinelDrive.setDefaultCommand(&sentinelAutoDrive);
    turret0::frictionWheels.setDefaultCommand(&turret0::spinFrictionWheels);
    turret1::frictionWheels.setDefaultCommand(&turret1::spinFrictionWheels);
    turret0::turretSubsystem.setDefaultCommand(&turret0::turretCVCommand);
    turret1::turretSubsystem.setDefaultCommand(&turret1::turretCVCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSentinelCommands(aruwsrc::Drivers *drivers)
{
    sentinelRequestHandler.attachSelectNewRobotMessageHandler(selectNewRobotMessageHandler);
    sentinelRequestHandler.attachTargetNewQuadrantMessageHandler(targetNewQuadrantMessageHandler);
    drivers->refSerial.attachRobotToRobotMessageHandler(
        aruwsrc::communication::serial::SENTINEL_REQUEST_ROBOT_ID,
        &sentinelRequestHandler);
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
    drivers->commandScheduler.setSafeDisconnectFunction(
        &sentinel_control::remoteSafeDisconnectFunction);
    sentinel_control::initializeSubsystems();
    sentinel_control::registerSentinelSubsystems(drivers);
    sentinel_control::setDefaultSentinelCommands(drivers);
    sentinel_control::startSentinelCommands(drivers);
    sentinel_control::registerSentinelIoMappings(drivers);
}
}  // namespace aruwsrc::control

#endif
