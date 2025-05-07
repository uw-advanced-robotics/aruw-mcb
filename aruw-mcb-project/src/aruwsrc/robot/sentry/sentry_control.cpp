/*
 * Copyright (c) 2022-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#if defined(TARGET_SENTRY_ECLIPSE)
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/double_dji_motor.hpp"

#include "aruwsrc/communication/mcb-lite/motor/virtual_dji_motor.hpp"
#include "aruwsrc/communication/mcb-lite/motor/virtual_double_dji_motor.hpp"
#include "aruwsrc/communication/mcb-lite/virtual_can_encoder.hpp"
#include "aruwsrc/communication/mcb-lite/virtual_current_sensor.hpp"
#include "aruwsrc/control/agitator/constant_velocity_agitator_command.hpp"
#include "aruwsrc/control/agitator/constants/agitator_constants.hpp"
#include "aruwsrc/control/agitator/unjam_spoke_agitator_command.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/aruco/aruco_reset_subsystem.hpp"
#include "aruwsrc/control/auto-aim/auto_aim_fire_rate_reselection_manager.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "aruwsrc/control/chassis/half_swerve_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/sentry/auto_nav_beyblade_command.hpp"
#include "aruwsrc/control/chassis/swerve_module.hpp"
#include "aruwsrc/control/chassis/swerve_module_config.hpp"
#include "aruwsrc/control/chassis/x_drive_chassis_subsystem.hpp"
#include "aruwsrc/control/client-display/client_display_command.hpp"
#include "aruwsrc/control/client-display/client_display_subsystem.hpp"
#include "aruwsrc/control/client-display/indicators/circle_crosshair.hpp"
#include "aruwsrc/control/client-display/indicators/image_indicator.hpp"
#include "aruwsrc/control/governor/fire_rate_limit_governor.hpp"
#include "aruwsrc/control/governor/friction_wheels_on_governor.hpp"
#include "aruwsrc/control/governor/heat_limit_governor.hpp"
#include "aruwsrc/control/governor/match_running_governor.hpp"
#include "aruwsrc/control/governor/ref_system_projectile_launched_governor.hpp"
#include "aruwsrc/control/launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/cv/sentry_turret_cv_command.hpp"
#include "aruwsrc/control/turret/yaw_turret_subsystem.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/sentry/sentry_auto_aim_launch_timer.hpp"
#include "aruwsrc/robot/sentry/sentry_ballistics_solver.hpp"
#include "aruwsrc/robot/sentry/sentry_beyblade_command.hpp"
#include "aruwsrc/robot/sentry/sentry_chassis_constants.hpp"
#include "aruwsrc/robot/sentry/sentry_chassis_world_yaw_observer.hpp"
#include "aruwsrc/robot/sentry/sentry_control_operator_interface.hpp"
#include "aruwsrc/robot/sentry/sentry_imu_calibrate_command.hpp"
#include "aruwsrc/robot/sentry/sentry_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_manual_drive_command.hpp"
#include "aruwsrc/robot/sentry/sentry_minor_cv_on_target_governor.hpp"
#include "aruwsrc/robot/sentry/sentry_transform_adapter.hpp"
#include "aruwsrc/robot/sentry/sentry_transform_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_constants.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_major_world_relative_yaw_controller.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_minor_subsystem.hpp"
#include "aruwsrc/robot/sentry/turret_major_control_command.hpp"
#include "aruwsrc/robot/sentry/turret_minor_control_command.hpp"

using namespace tap::algorithms;
using namespace tap::control;
using namespace tap::communication::serial;
using namespace tap::control::governor;
using namespace tap::control::setpoint;

using namespace aruwsrc::agitator;
using namespace aruwsrc::sentry;
using namespace aruwsrc::control::agitator;
using namespace aruwsrc::sentry::chassis;
using namespace aruwsrc::control::governor;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::control::sentry;
using namespace aruwsrc::control::turret::sentry;
using namespace aruwsrc::control::turret::algorithms;
using namespace aruwsrc::virtualMCB;
using namespace aruwsrc::control;
using namespace aruwsrc::control::client_display;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace sentry_control
{
MatchRunningGovernor matchRunningGovernor(drivers()->refSerial);

aruwsrc::virtualMCB::VirtualCanEncoder turretMajorYawEncoder(
    drivers(),
    tap::encoder::CanEncoderId::ID0,
    &drivers()->chassisMcbLite,
    tap::can::CanBus::CAN_BUS2,
    false,
    1.0f,
    turretMajor::YAW_MOTOR_CONFIG.startEncoderValue
);

aruwsrc::virtualMCB::VirtualDjiMotor turretMajorYawMotor(
    drivers(),
    tap::motor::MOTOR5,
    turretMajor::CAN_BUS_MOTOR,
    &drivers()->chassisMcbLite,
    false,
    "Major Yaw Turret",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508,
    0,
    &turretMajorYawEncoder);

struct TurretMinorMotors
{
    tap::motor::DjiMotor yawMotor;
    tap::motor::DjiMotor pitchMotor;
    TurretMotorConfig yawMotorConfig;
    TurretMotorConfig pitchMotorConfig;
};

TurretMinorMotors turretLeftMotors{
    .yawMotor = tap::motor::DjiMotor(
        drivers(),
        turretLeft::YAW_MOTOR_ID,
        turretLeft::CAN_BUS_MOTORS,
        false,
        "Left Minor Yaw Turret",
        true,
        1.0,
        turretLeft::YAW_MOTOR_CONFIG.startEncoderValue),

    .pitchMotor = tap::motor::DjiMotor(
        drivers(),
        turretLeft::PITCH_MOTOR_ID,
        turretLeft::CAN_BUS_MOTORS,
        true,
        "Left Minor Pitch Turret",
        true,
        1.0,
        turretLeft::PITCH_MOTOR_CONFIG.startEncoderValue),

    .yawMotorConfig = turretLeft::YAW_MOTOR_CONFIG,
    .pitchMotorConfig = turretLeft::PITCH_MOTOR_CONFIG

};

TurretMinorMotors turretRightMotors{
    .yawMotor = tap::motor::DjiMotor(
        drivers(),
        turretRight::YAW_MOTOR_ID,
        turretRight::CAN_BUS_MOTORS,
        false,
        "Right Minor Yaw Turret",
        true,
        1.0,
        turretRight::YAW_MOTOR_CONFIG.startEncoderValue),

    .pitchMotor = tap::motor::DjiMotor(
        drivers(),
        turretRight::PITCH_MOTOR_ID,
        turretRight::CAN_BUS_MOTORS,
        true,
        "Right Minor Pitch Turret",
        true,
        1.0,
        turretRight::PITCH_MOTOR_CONFIG.startEncoderValue),

    .yawMotorConfig = turretRight::YAW_MOTOR_CONFIG,
    .pitchMotorConfig = turretRight::PITCH_MOTOR_CONFIG

};

inline aruwsrc::can::TurretMCBCanComm &getTurretMCBCanComm1()
{
    return drivers()->turretMCBCanCommBus1;
}
inline aruwsrc::can::TurretMCBCanComm &getTurretMCBCanComm2()
{
    return drivers()->turretMCBCanCommBus2;
}

// /* define subsystems --------------------------------------------------------*/
YawTurretSubsystem turretMajor(*drivers(), turretMajorYawMotor, turretMajor::YAW_MOTOR_CONFIG);

SentryTurretMinorSubsystem turretLeft(
    *drivers(),
    turretLeftMotors.pitchMotor,
    turretLeftMotors.yawMotor,
    turretLeftMotors.pitchMotorConfig,
    turretLeftMotors.yawMotorConfig,
    &drivers()->turretMCBCanCommBus2,  // @todo: figure out how to put this in config
    turretLeft::turretID);

SentryTurretMinorSubsystem turretRight(
    *drivers(),
    turretRightMotors.pitchMotor,
    turretRightMotors.yawMotor,
    turretRightMotors.pitchMotorConfig,
    turretRightMotors.yawMotorConfig,
    &drivers()->turretMCBCanCommBus1,  // @todo: figure out how to put this in config
    turretRight::turretID);

SentryChassisWorldYawObserver chassisYawObserver(drivers()->turretMajorMcbLite.imu, turretMajor);

struct TurretMinorChassisControllers
{
    ChassisFramePitchTurretController pitchController;
    ChassisFrameYawTurretController yawController;
};

// @todo make controllers part of subsystem
TurretMinorChassisControllers turretLeftChassisControllers{
    .pitchController = ChassisFramePitchTurretController(
        turretLeft.pitchMotor,
        minorPidConfigs::PITCH_PID_CONFIG_CHASSIS_FRAME),
    .yawController = ChassisFrameYawTurretController(
        turretLeft.yawMotor,
        minorPidConfigs::YAW_PID_CONFIG_CHASSIS_FRAME),
};

TurretMinorChassisControllers turretRightChassisControllers{
    .pitchController = ChassisFramePitchTurretController(
        turretRight.pitchMotor,
        minorPidConfigs::PITCH_PID_CONFIG_CHASSIS_FRAME),
    .yawController = ChassisFrameYawTurretController(
        turretRight.yawMotor,
        minorPidConfigs::YAW_PID_CONFIG_CHASSIS_FRAME),
};

VirtualDjiMotor rightFrontMotor(
    drivers(),
    MOTOR1,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->chassisMcbLite),
    false,
    "Right Front Motor",
    false,
    1.0f / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

VirtualDjiMotor leftFrontMotor(
    drivers(),
    MOTOR2,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->chassisMcbLite),
    false,
    "Left Front Motor",
    false,
    1.0f / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

VirtualDjiMotor leftBackMotor(
    drivers(),
    MOTOR3,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->chassisMcbLite),
    false,
    "Left Back Motor",
    false,
    1.0f / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

VirtualDjiMotor rightBackMotor(
    drivers(),
    MOTOR4,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->chassisMcbLite),
    false,
    "Right Back Motor",
    false,
    1.0f / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

aruwsrc::virtualMCB::VirtualCurrentSensor currentSensor(
    {&drivers()->chassisMcbLite.analog,
     aruwsrc::chassis::CURRENT_SENSOR_PIN,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_MV_PER_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_ZERO_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA});

aruwsrc::chassis::XDriveChassisSubsystem chassis(
    drivers(),
    &currentSensor,
    leftFrontMotor,
    leftBackMotor,
    rightFrontMotor,
    rightBackMotor,
    {.kp = 5.0f, .ki = 0.0f, .kd = 0.0f, .maxOutput = 16000.0f, .errDeadzone = 100.0f});

aruwsrc::algorithms::odometry::TwoDeadwheelOdometryObserver deadwheels(
    &leftFrontMotor,
    &leftBackMotor,
    DEADWHEEL_RADIUS);

SentryKFOdometry2DSubsystem odometrySubsystem(
    *drivers(),
    deadwheels,
    chassisYawObserver,
    drivers()->chassisMcbLite.imu,
    INITIAL_CHASSIS_POSITION_X,
    INITIAL_CHASSIS_POSITION_Y,
    CENTER_TO_WHEELBASE_RADIUS);

SentryTransforms transformer(
    odometrySubsystem,
    turretMajor,
    turretLeft,
    turretRight,
    {.turretMinorOffset = TURRET_MINOR_OFFSET});

SentryTransformSubystem transformerSubsystem(*drivers(), transformer);
SentryTransformAdapter transformAdapter(transformer);

aruwsrc::control::aruco::ArucoResetSubsystem arucoResetSubsystem(
    drivers(),
    drivers()->visionCoprocessor,
    odometrySubsystem,
    transformAdapter);

aruwsrc::chassis::ChassisAutoNavController autoNavController(
    *drivers(),
    chassis,
    drivers()->visionCoprocessor,
    transformer.getWorldToChassis(),
    aruwsrc::sentry::chassis::beybladeConfig);

SmoothPid turretMajorYawPosPid(turretMajor::worldFrameCascadeController::YAW_POS_PID_CONFIG);
SmoothPid turretMajorYawVelPid(turretMajor::worldFrameCascadeController::YAW_VEL_PID_CONFIG);

struct TurretMinorWorldControllers
{
    WorldFramePitchTurretImuCascadePidTurretController pitchController;
    WorldFrameYawTurretImuCascadePidTurretController yawController;
};

// // @todo surely there's a better way to construct this
SmoothPid turretLeftWorldPitchVelPid(minorPidConfigs::PITCH_PID_CONFIG_WORLD_FRAME_VEL);
SmoothPid turretLeftWorldPitchPosPid(minorPidConfigs::PITCH_PID_CONFIG_WORLD_FRAME_POS);
SmoothPid turretLeftWorldYawVelPid(minorPidConfigs::YAW_PID_CONFIG_WORLD_FRAME_VEL);
SmoothPid turretLeftWorldYawPosPid(minorPidConfigs::YAW_PID_CONFIG_WORLD_FRAME_POS);
SmoothPid turretRightWorldPitchVelPid(minorPidConfigs::PITCH_PID_CONFIG_WORLD_FRAME_VEL);
SmoothPid turretRightWorldPitchPosPid(minorPidConfigs::PITCH_PID_CONFIG_WORLD_FRAME_POS);
SmoothPid turretRightWorldYawVelPid(minorPidConfigs::YAW_PID_CONFIG_WORLD_FRAME_VEL);
SmoothPid turretRightWorldYawPosPid(minorPidConfigs::YAW_PID_CONFIG_WORLD_FRAME_POS);

TurretMinorWorldControllers turretRightWorldControllers{
    .pitchController = WorldFramePitchTurretImuCascadePidTurretController(
        drivers()->turretMCBCanCommBus1,
        turretRight.pitchMotor,
        turretRightWorldPitchPosPid,
        turretRightWorldPitchVelPid),

    .yawController = WorldFrameYawTurretImuCascadePidTurretController(
        drivers()->turretMCBCanCommBus1,
        turretRight.yawMotor,
        turretRightWorldYawPosPid,
        turretRightWorldYawVelPid)

};

TurretMinorWorldControllers turretLeftWorldControllers{
    .pitchController = WorldFramePitchTurretImuCascadePidTurretController(
        drivers()->turretMCBCanCommBus2,
        turretLeft.pitchMotor,
        turretLeftWorldPitchPosPid,
        turretLeftWorldPitchVelPid),

    .yawController = WorldFrameYawTurretImuCascadePidTurretController(
        drivers()->turretMCBCanCommBus2,
        turretLeft.yawMotor,
        turretLeftWorldYawPosPid,
        turretLeftWorldYawVelPid)

};

TurretMajorWorldFrameController turretMajorWorldYawController(  // @todo rename
    transformer.getWorldToTurretMajor(),
    chassis,
    turretMajor.getMutableMotor(),
    drivers()->turretMajorMcbLite.imu,
    turretLeft,
    turretRight,
    turretMajorYawPosPid,
    turretMajorYawVelPid,
    turretMajor::MAX_VEL_ERROR_INPUT,
    turretMajor::TURRET_MINOR_TORQUE_RATIO,
    turretMajor::FEEDFORWARD_GAIN);

ChassisFrameYawTurretController turretMajorChassisYawController(
    turretMajor.getMutableMotor(),
    turretMajor::chassisFrameController::YAW_PID_CONFIG);

// Friction Wheels
aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    turretLeftFrictionWheels(
        drivers(),
        aruwsrc::control::launcher::LEFT_MOTOR_ID,
        aruwsrc::control::launcher::RIGHT_MOTOR_ID,
        turretLeft::CAN_BUS_MOTORS,
        &getTurretMCBCanComm2(),
        turretLeft::barrelID);

aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    turretRightFrictionWheels(
        drivers(),
        aruwsrc::control::launcher::LEFT_MOTOR_ID,
        aruwsrc::control::launcher::RIGHT_MOTOR_ID,
        turretRight::CAN_BUS_MOTORS,
        &getTurretMCBCanComm1(),
        turretRight::barrelID);  // @todo idk what they actually are

// Agitators
VelocityAgitatorSubsystem turretLeftAgitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::turretLeft::AGITATOR_CONFIG);

VelocityAgitatorSubsystem turretRightAgitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::turretRight::AGITATOR_CONFIG);

// ballistics solvers
SentryBallisticsSolver turretRightSolver(
    drivers()->visionCoprocessor,
    transformer,
    turretRightFrictionWheels,
    turretMajor,
    turretRight::DEFAULT_LAUNCH_SPEED,
    0.f,  // turret minor pitch offset
    TURRET_MINOR_OFFSET,
    turretRight.getTurretID());

SentryAutoAimLaunchTimer autoAimLaunchTimerTurretRight(
    aruwsrc::control::launcher::AGITATOR_TYPICAL_DELAY_MICROSECONDS,
    &drivers()->visionCoprocessor,
    &turretRightSolver);

SentryBallisticsSolver turretLeftSolver(
    drivers()->visionCoprocessor,
    transformer,
    turretLeftFrictionWheels,
    turretMajor,
    turretLeft::DEFAULT_LAUNCH_SPEED,
    0.f,  // turret minor pitch offset
    TURRET_MINOR_OFFSET,
    turretLeft.getTurretID());

SentryAutoAimLaunchTimer autoAimLaunchTimerTurretLeft(
    aruwsrc::control::launcher::AGITATOR_TYPICAL_DELAY_MICROSECONDS,
    &drivers()->visionCoprocessor,
    &turretLeftSolver);

/* define commands ----------------------------------------------------------*/
aruwsrc::chassis::AutoNavBeybladeCommand autoNavBeybladeCommand(
    *drivers(),
    chassis,
    autoNavController,
    false);

TurretMajorSentryControlCommand majorManualCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    turretMajor,
    turretMajorWorldYawController,
    MAJOR_USER_YAW_INPUT_SCALAR);

TurretMinorSentryControlCommand turretLeftManualCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    turretLeft,
    turretLeftWorldControllers.yawController,
    turretLeftWorldControllers.pitchController,
    MINOR_USER_YAW_INPUT_SCALAR,
    MINOR_USER_PITCH_INPUT_SCALAR);

TurretMinorSentryControlCommand turretRightManualCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    turretRight,
    turretRightWorldControllers.yawController,
    turretRightWorldControllers.pitchController,
    MINOR_USER_YAW_INPUT_SCALAR,
    MINOR_USER_PITCH_INPUT_SCALAR);

// Chassis beyblade
aruwsrc::sentry::SentryBeybladeCommand beybladeCommand(
    drivers(),
    &chassis,
    &turretMajor.getReadOnlyMotor(),
    drivers()->controlOperatorInterface,
    transformer.getWorldToChassis(),
    aruwsrc::sentry::chassis::beybladeConfig);

aruwsrc::control::sentry::SentryManualDriveCommand chassisDriveCommand(
    drivers(),
    &(drivers()->controlOperatorInterface),
    &chassis);

imu::SentryImuCalibrateCommand imuCalibrateCommand(
    drivers(),
    {
        {
            &drivers()->turretMCBCanCommBus2,
            &turretLeft,
            &turretLeftChassisControllers.yawController,
            &turretLeftChassisControllers.pitchController,
            true,
        },
        {
            &drivers()->turretMCBCanCommBus1,
            &turretRight,
            &turretRightChassisControllers.yawController,
            &turretRightChassisControllers.pitchController,
            true,
        },
    },
    turretMajor,
    turretMajorChassisYawController,
    chassis,
    chassisYawObserver,
    odometrySubsystem,
    drivers()->turretMajorMcbLite,
    drivers()->chassisMcbLite);

SentryTurretCVCommand::TurretConfig turretLeftCVConfig(
    turretLeft,
    turretLeftWorldControllers.yawController,
    turretLeftWorldControllers.pitchController,
    turretLeftSolver);

SentryTurretCVCommand::TurretConfig turretRightCVConfig(
    turretRight,
    turretRightWorldControllers.yawController,
    turretRightWorldControllers.pitchController,
    turretRightSolver);

SentryTurretCVCommand turretCVCommand(
    drivers()->visionCoprocessor,
    turretMajor,
    turretMajorWorldYawController,
    turretLeftCVConfig,
    turretRightCVConfig,
    transformer);

// LEFT shooting ======================

// spin friction wheels commands
aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand
turretLeftFrictionWheelSpinCommand(
    drivers(),
    &turretLeftFrictionWheels,
    aruwsrc::control::launcher::LAUNCHER_SPEED,
    false,
    turretLeft::barrelID);

aruwsrc::control::launcher::
    FrictionWheelSpinRefLimitedCommand stopTurretLeftFrictionWheelSpinCommand(
        drivers(),
        &turretLeftFrictionWheels,
        0.0f,
        true,
        turretLeft::barrelID);

// Agitator commands (turret left)
ConstantVelocityAgitatorCommand turretLeftRotateAgitator(
    turretLeftAgitator,
    constants::AGITATOR_ROTATE_CONFIG);
UnjamSpokeAgitatorCommand turretLeftUnjamAgitator(
    turretLeftAgitator,
    constants::AGITATOR_UNJAM_CONFIG);
MoveUnjamIntegralComprisedCommand turretLeftRotateAndUnjamAgitator(
    *drivers(),
    turretLeftAgitator,
    turretLeftRotateAgitator,
    turretLeftUnjamAgitator);

AutoAimFireRateReselectionManager fireRateReselectionManagerTurretLeft(
    *drivers(),
    drivers()->visionCoprocessor,
    drivers()->commandScheduler,
    turretCVCommand,
    turretLeft::turretID);

FireRateLimitGovernor fireRateLimitGovernorTurretLeft(fireRateReselectionManagerTurretLeft);

// rotates agitator with heat limiting applied
HeatLimitGovernor heatLimitGovernorTurretLeft(
    *drivers(),
    turretLeft::barrelID,
    constants::HEAT_LIMIT_BUFFER);

// rotates agitator when aiming at target and within heat limit
SentryMinorCvOnTargetGovernor cvOnTargetGovernorTurretLeft(
    drivers(),
    drivers()->visionCoprocessor,
    turretCVCommand,
    autoAimLaunchTimerTurretLeft,
    SentryCvOnTargetGovernorMode::ON_TARGET_AND_GATED,
    turretLeft::turretID);

// TODO:: see if this actually does stuff, test later.
RefSystemProjectileLaunchedGovernor refSystemProjectileLaunchedGovernorTurretLeft(
    drivers()->refSerial,
    turretLeft::barrelID);

FrictionWheelsOnGovernor frictionWheelsOnGovernorTurretLeft(turretLeftFrictionWheels);

GovernorLimitedCommand<6> turretLeftRotateAndUnjamAgitatorWithHeatAndCVLimiting(
    {&turretLeftAgitator},
    turretLeftRotateAndUnjamAgitator,
    {&fireRateLimitGovernorTurretLeft,
     &heatLimitGovernorTurretLeft,
     &refSystemProjectileLaunchedGovernorTurretLeft,
     &frictionWheelsOnGovernorTurretLeft,
     &cvOnTargetGovernorTurretLeft,
     &matchRunningGovernor});

GovernorLimitedCommand<3> turretLeftAgitatorManualSpin(
    {&turretLeftAgitator},
    turretLeftRotateAndUnjamAgitator,
    {&heatLimitGovernorTurretLeft,
     &refSystemProjectileLaunchedGovernorTurretLeft,
     &frictionWheelsOnGovernorTurretLeft});

// RIGHT shooting ======================

// spin friction wheels commands
aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand
turretRightFrictionWheelSpinCommand(
    drivers(),
    &turretRightFrictionWheels,
    aruwsrc::control::launcher::LAUNCHER_SPEED,
    false,
    turretRight::barrelID);

aruwsrc::control::launcher::
    FrictionWheelSpinRefLimitedCommand stopTurretRightFrictionWheelSpinCommand(
        drivers(),
        &turretRightFrictionWheels,
        0.0f,
        true,
        turretRight::barrelID);

// Agitator commands (turret Right)
ConstantVelocityAgitatorCommand turretRightRotateAgitator(
    turretRightAgitator,
    constants::AGITATOR_ROTATE_CONFIG);
UnjamSpokeAgitatorCommand turretRightUnjamAgitator(
    turretRightAgitator,
    constants::AGITATOR_UNJAM_CONFIG);
MoveUnjamIntegralComprisedCommand turretRightRotateAndUnjamAgitator(
    *drivers(),
    turretRightAgitator,
    turretRightRotateAgitator,
    turretRightUnjamAgitator);

AutoAimFireRateReselectionManager fireRateReselectionManagerTurretRight(
    *drivers(),
    drivers()->visionCoprocessor,
    drivers()->commandScheduler,
    turretCVCommand,
    turretRight::turretID);

FireRateLimitGovernor fireRateLimitGovernorTurretRight(fireRateReselectionManagerTurretRight);

// rotates agitator with heat limiting applied
HeatLimitGovernor heatLimitGovernorTurretRight(
    *drivers(),
    turretRight::barrelID,
    constants::HEAT_LIMIT_BUFFER);

// rotates agitator when aiming at target and within heat limit
SentryMinorCvOnTargetGovernor cvOnTargetGovernorTurretRight(
    drivers(),
    drivers()->visionCoprocessor,
    turretCVCommand,
    autoAimLaunchTimerTurretRight,
    SentryCvOnTargetGovernorMode::ON_TARGET_AND_GATED,
    turretRight::turretID);

RefSystemProjectileLaunchedGovernor refSystemProjectileLaunchedGovernorTurretRight(
    drivers()->refSerial,
    turretRight::barrelID);

FrictionWheelsOnGovernor frictionWheelsOnGovernorTurretRight(turretRightFrictionWheels);

GovernorLimitedCommand<6> turretRightRotateAndUnjamAgitatorWithHeatAndCVLimiting(
    {&turretRightAgitator},
    turretRightRotateAndUnjamAgitator,
    {&fireRateLimitGovernorTurretRight,
     &heatLimitGovernorTurretRight,
     &refSystemProjectileLaunchedGovernorTurretRight,
     &frictionWheelsOnGovernorTurretRight,
     &cvOnTargetGovernorTurretRight,
     &matchRunningGovernor});

GovernorLimitedCommand<3> turretRightAgitatorManualSpin(
    {&turretRightAgitator},
    turretRightRotateAndUnjamAgitator,
    {&heatLimitGovernorTurretRight,
     &refSystemProjectileLaunchedGovernorTurretRight,
     &frictionWheelsOnGovernorTurretRight});

/* define client display / HUD related items --------------------------------*/

// This shit is currently banned by DJI, but left for a hopeful future
ClientDisplaySubsystem clientDisplay(drivers());
tap::communication::serial::RefSerialTransmitter refSerialTransmitter(drivers());

CircleCrosshair circleCrosshair(refSerialTransmitter);
ImageIndicator imageIndicator(refSerialTransmitter);

std::vector<HudIndicator *> indicators = {&imageIndicator, &circleCrosshair};

ClientDisplayCommand clientDisplayCommand(*drivers(), clientDisplay, indicators);

/* define command mappings --------------------------------------------------*/

HoldCommandMapping rightUp(
    drivers(),
    {&turretLeftFrictionWheelSpinCommand, &turretRightFrictionWheelSpinCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

// auto nav + auto aim + cv gated fire
HoldCommandMapping leftUpRightUp(
    drivers(),
    {&autoNavBeybladeCommand, &turretCVCommand},
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::UP));

HoldRepeatCommandMapping leftUpRightUpAg(
    drivers(),
    {&turretLeftRotateAndUnjamAgitatorWithHeatAndCVLimiting,
     &turretRightRotateAndUnjamAgitatorWithHeatAndCVLimiting},
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::UP),
    false);

// auto nav + auto aim
HoldCommandMapping leftUpRightMid(
    drivers(),
    {&autoNavBeybladeCommand, &turretCVCommand},
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::MID));

// imu calibrate
HoldCommandMapping leftUpRightDown(
    drivers(),
    {&imuCalibrateCommand},
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::DOWN));

// manual aim and shoot
HoldCommandMapping leftMidRightUp(
    drivers(),
    {&turretLeftManualCommand, &turretRightManualCommand},
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::UP));

// manual aim and shoot
HoldRepeatCommandMapping leftMidRightUpAg(
    drivers(),
    {&turretLeftAgitatorManualSpin, &turretRightAgitatorManualSpin},
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::UP),
    false);

// auto drive & auto aim
HoldCommandMapping leftMidRightMid(
    drivers(),
    {&majorManualCommand,
     &turretLeftManualCommand,
     &turretRightManualCommand,
     &autoNavBeybladeCommand},
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::MID));

// manual aim
HoldCommandMapping leftMidRightDown(
    drivers(),
    {
        &majorManualCommand,
        &turretLeftManualCommand,
        &turretRightManualCommand,
    },
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::DOWN));

// manual drive, auto aim, cv-gated fire
HoldCommandMapping leftDownRightUp(
    drivers(),
    {&chassisDriveCommand, &turretCVCommand},
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::UP));

HoldRepeatCommandMapping leftDownRightUpAg(
    drivers(),
    {&turretLeftRotateAndUnjamAgitatorWithHeatAndCVLimiting,
     &turretRightRotateAndUnjamAgitatorWithHeatAndCVLimiting},
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::UP),
    false);

// manual drive & auto aim
HoldCommandMapping leftDownRightMid(
    drivers(),
    {&chassisDriveCommand, &turretCVCommand},
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::MID));

// manual drive
HoldCommandMapping leftDownRightDown(
    drivers(),
    {&chassisDriveCommand},
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::DOWN));

// Restart HUD
PressCommandMapping bCtrlPressed(
    drivers(),
    {&clientDisplayCommand},
    RemoteMapState({Remote::Key::CTRL, Remote::Key::B}));

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());
/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    chassis.initialize();
    turretLeft.initialize();
    turretRight.initialize();
    turretMajor.initialize();
    odometrySubsystem.initialize();
    transformerSubsystem.initialize();
    arucoResetSubsystem.initialize();

    // rightOmni.initialize();
    // leftOmni.initialize();

    turretLeftFrictionWheels.initialize();
    turretRightFrictionWheels.initialize();

    turretLeftAgitator.initialize();
    turretRightAgitator.initialize();

    clientDisplay.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerSentrySubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&turretMajor);
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&turretLeft);
    drivers->commandScheduler.registerSubsystem(&turretRight);
    drivers->commandScheduler.registerSubsystem(&odometrySubsystem);
    drivers->commandScheduler.registerSubsystem(&transformerSubsystem);
    drivers->commandScheduler.registerSubsystem(&arucoResetSubsystem);
    drivers->commandScheduler.registerSubsystem(&clientDisplay);

    drivers->commandScheduler.registerSubsystem(&turretLeftFrictionWheels);
    drivers->commandScheduler.registerSubsystem(&turretRightFrictionWheels);
    drivers->commandScheduler.registerSubsystem(&turretLeftAgitator);
    drivers->commandScheduler.registerSubsystem(&turretRightAgitator);

    drivers->visionCoprocessor.attachTransformer(&transformAdapter);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSentryCommands(Drivers *)
{
    chassis.setDefaultCommand(&chassisDriveCommand);
    turretMajor.setDefaultCommand(&majorManualCommand);
    turretLeft.setDefaultCommand(&turretLeftManualCommand);
    turretRight.setDefaultCommand(&turretRightManualCommand);

    turretLeftFrictionWheels.setDefaultCommand(&stopTurretLeftFrictionWheelSpinCommand);
    turretRightFrictionWheels.setDefaultCommand(&stopTurretRightFrictionWheelSpinCommand);

    clientDisplay.setDefaultCommand(&clientDisplayCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSentryCommands(Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&imuCalibrateCommand);
    drivers->plateHitTracker.attachTransformer(&transformAdapter);
}

/* register io mappings here ------------------------------------------------*/
void registerSentryIoMappings(Drivers *drivers)
{
    // commands with higher priority must be added later
    // friction wheels spin (separated due to dumb design in command mapper system)
    drivers->commandMapper.addMap(&rightUp);

    drivers->commandMapper.addMap(&leftDownRightMid);  // manual drive & auto aim
    drivers->commandMapper.addMap(&leftDownRightUp);   // manual drive, auto aim, gated-fire
    drivers->commandMapper.addMap(&leftDownRightUpAg);
    drivers->commandMapper.addMap(&leftDownRightDown);  // manual drive

    drivers->commandMapper.addMap(&leftMidRightUp);  // manual aim and shoot
    drivers->commandMapper.addMap(&leftMidRightUpAg);
    drivers->commandMapper.addMap(&leftMidRightMid);   // auto drive & auto aim
    drivers->commandMapper.addMap(&leftMidRightDown);  // manual aim

    drivers->commandMapper.addMap(&leftUpRightMid);  // auto nav + auto aim
    drivers->commandMapper.addMap(&leftUpRightUp);   // auto nav + auto aim + cv gated fire
    drivers->commandMapper.addMap(&leftUpRightUpAg);
    drivers->commandMapper.addMap(&leftUpRightDown);  // imu calibrate
}

}  // namespace sentry_control

namespace aruwsrc::sentry
{
void initSubsystemCommands(aruwsrc::sentry::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &sentry_control::remoteSafeDisconnectFunction);
    sentry_control::initializeSubsystems();
    sentry_control::registerSentrySubsystems(drivers);
    sentry_control::setDefaultSentryCommands(drivers);
    sentry_control::startSentryCommands(drivers);
    sentry_control::registerSentryIoMappings(drivers);
}
}  // namespace aruwsrc::sentry

// #ifndef PLATFORM_HOSTED
// imu::ImuCalibrateCommand *getImuCalibrateCommand() { return &sentry_control::imuCalibrateCommand;
// } #endif

#endif
