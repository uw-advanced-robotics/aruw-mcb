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

#if defined(TARGET_SENTINEL_2021)

#include "tap/control/command_mapper.hpp"
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/governor/governor_with_fallback_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/setpoint/commands/unjam_integral_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/motor/double_dji_motor.hpp"

#include "agitator/constants/agitator_constants.hpp"
#include "agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/otto_velocity_odometry_2d_subsystem.hpp"
#include "aruwsrc/communication/serial/sentinel_request_handler.hpp"
#include "aruwsrc/communication/serial/sentinel_request_message_types.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "governor/cv_has_target_governor.hpp"
#include "governor/friction_wheels_on_governor.hpp"
#include "governor/heat_limit_governor.hpp"
#include "launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "launcher/launcher_constants.hpp"
#include "launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "sentinel/drive/sentinel_auto_drive_comprised_command.hpp"
#include "sentinel/drive/sentinel_drive_manual_command.hpp"
#include "sentinel/drive/sentinel_drive_subsystem.hpp"
#include "turret/algorithms/chassis_frame_turret_controller.hpp"
#include "turret/constants/turret_constants.hpp"
#include "turret/cv/turret_cv_command.hpp"
#include "turret/cv/turret_scan_command.hpp"
#include "turret/sentinel_turret_subsystem.hpp"
#include "turret/user/turret_quick_turn_command.hpp"
#include "turret/user/turret_user_control_command.hpp"

using namespace tap::control::setpoint;
using namespace tap::control::governor;
using namespace aruwsrc::agitator;
using namespace aruwsrc::control::sentinel::drive;
using namespace tap::gpio;
using namespace aruwsrc::control::agitator;
using namespace aruwsrc::control;
using namespace aruwsrc::control::governor;
using namespace tap::control;
using namespace tap::motor;
using namespace aruwsrc::control::turret;
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
static constexpr Digital::InputPin LEFT_LIMIT_SWITCH = Digital::InputPin::C;
static constexpr Digital::InputPin RIGHT_LIMIT_SWITCH = Digital::InputPin::B;

aruwsrc::communication::serial::SentinelRequestHandler sentinelRequestHandler(drivers());

/* define subsystems --------------------------------------------------------*/
VelocityAgitatorSubsystem agitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::AGITATOR_CONFIG);

SentinelDriveSubsystem sentinelDrive(drivers(), LEFT_LIMIT_SWITCH, RIGHT_LIMIT_SWITCH);

aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    frictionWheels(
        drivers(),
        aruwsrc::control::launcher::LEFT_MOTOR_ID,
        aruwsrc::control::launcher::RIGHT_MOTOR_ID,
        aruwsrc::control::launcher::CAN_BUS_MOTORS,
        nullptr,
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

// Note: motor "one" is right, "two" is left
tap::motor::DjiMotor pitchMotor(
    drivers(),
    tap::motor::MOTOR5,
    aruwsrc::control::turret::CAN_BUS_MOTORS,
    false,
    "Pitch Turret");
tap::motor::DjiMotor yawMotor(
    drivers(),
    tap::motor::MOTOR6,
    aruwsrc::control::turret::CAN_BUS_MOTORS,
    true,
    "Yaw Turret");
SentinelTurretSubsystem turretSubsystem(
    drivers(),
    &pitchMotor,
    &yawMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG,
    nullptr);

OttoVelocityOdometry2DSubsystem odometrySubsystem(drivers(), turretSubsystem, &sentinelDrive);

/* define commands ----------------------------------------------------------*/
MoveIntegralCommand rotateAgitator(agitator, constants::AGITATOR_ROTATE_CONFIG);

UnjamIntegralCommand unjamAgitator(agitator, constants::AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
    *drivers(),
    agitator,
    rotateAgitator,
    unjamAgitator);

// rotates agitator if friction wheels are spinning fast
FrictionWheelsOnGovernor frictionWheelsOnGovernor(frictionWheels);

// rotates agitator with heat limiting applied
HeatLimitGovernor heatLimitGovernor(
    *drivers(),
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1,
    constants::HEAT_LIMIT_BUFFER);
GovernorLimitedCommand<2> rotateAndUnjamAgitatorWithHeatLimiting(
    {&agitator},
    rotateAndUnjamAgitator,
    {&heatLimitGovernor, &frictionWheelsOnGovernor});

// Two identical drive commands since you can't map an identical command to two different mappings
SentinelDriveManualCommand sentinelDriveManual1(drivers(), &sentinelDrive);
SentinelDriveManualCommand sentinelDriveManual2(drivers(), &sentinelDrive);

FrictionWheelSpinRefLimitedCommand spinFrictionWheels(
    drivers(),
    &frictionWheels,
    30.0f,
    true,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

FrictionWheelSpinRefLimitedCommand stopFrictionWheels(
    drivers(),
    &frictionWheels,
    0.0f,
    true,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

// turret controllers
algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController(
    turretSubsystem.pitchMotor,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
    turretSubsystem.yawMotor,
    chassis_rel::YAW_PID_CONFIG);

// turret commands

user::TurretUserControlCommand turretManual(
    drivers(),
    &turretSubsystem,
    &chassisFrameYawTurretController,
    &chassisFramePitchTurretController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR,
    0);

cv::TurretCVCommand turretCVCommand(
    drivers(),
    &turretSubsystem,
    &chassisFrameYawTurretController,
    &chassisFramePitchTurretController,
    odometrySubsystem,
    frictionWheels,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR,
    29.5f);

cv::TurretScanCommand turretScanCommand(
    turretSubsystem,
    chassisFrameYawTurretController,
    chassisFramePitchTurretController,
    TURRET_SCAN_CONFIG);

CvHasTargetGovernor cvHasTargetGovernor(drivers()->visionCoprocessor, 0);
tap::control::governor::GovernorWithFallbackCommand<1> turretCVWithScanFallback(
    {&turretSubsystem},
    turretCVCommand,
    turretScanCommand,
    {&cvHasTargetGovernor});

user::TurretQuickTurnCommand turretUturnCommand(&turretSubsystem, M_PI);

void selectNewRobotMessageHandler() { drivers()->visionCoprocessor.sendSelectNewTargetMessage(); }

void targetNewQuadrantMessageHandler()
{
    drivers()->commandScheduler.addCommand(&turretUturnCommand);
}

SentinelAutoDriveComprisedCommand sentinelAutoDrive(drivers(), &sentinelDrive);

/* define command mappings --------------------------------------------------*/

HoldCommandMapping rightSwitchDown(
    drivers(),
    {&stopFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&rotateAndUnjamAgitatorWithHeatLimiting},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    true);
HoldRepeatCommandMapping leftSwitchDown(
    drivers(),
    {&sentinelDriveManual1, &turretManual},
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
    odometrySubsystem.initialize();
}

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* register subsystems here ------------------
-------------------------------*/
void registerSentinelSubsystems(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&sentinelDrive);
    drivers->commandScheduler.registerSubsystem(&frictionWheels);
    drivers->commandScheduler.registerSubsystem(&turretSubsystem);
    drivers->commandScheduler.registerSubsystem(&odometrySubsystem);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSentinelCommands(aruwsrc::Drivers *drivers)
{
    sentinelDrive.setDefaultCommand(&sentinelAutoDrive);
    frictionWheels.setDefaultCommand(&spinFrictionWheels);
    turretSubsystem.setDefaultCommand(&turretCVWithScanFallback);
    drivers->visionCoprocessor.attachOdometryInterface(&odometrySubsystem);
    drivers->visionCoprocessor.attachTurretOrientationInterface(&turretSubsystem, 0);
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
}  // namespace aruwsrc
}  // namespace aruwsrc::control

#endif
