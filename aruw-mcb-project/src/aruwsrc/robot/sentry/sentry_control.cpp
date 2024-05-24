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

#if defined(TARGET_SENTRY_HYDRA)
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/double_dji_motor.hpp"

#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"
#include "aruwsrc/communication/mcb-lite/motor/virtual_dji_motor.hpp"
#include "aruwsrc/communication/mcb-lite/motor/virtual_double_dji_motor.hpp"
#include "aruwsrc/communication/mcb-lite/virtual_current_sensor.hpp"
#include "aruwsrc/control/agitator/constants/agitator_constants.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "aruwsrc/control/chassis/half_swerve_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/new_sentry/sentry_manual_drive_command.hpp"
#include "aruwsrc/control/chassis/swerve_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/swerve_module.hpp"
#include "aruwsrc/control/chassis/swerve_module_config.hpp"
#include "aruwsrc/control/launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/cv/sentry_turret_cv_command.hpp"
#include "aruwsrc/control/turret/yaw_turret_subsystem.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/sentry/sentry_beyblade_command.hpp"
#include "aruwsrc/robot/sentry/sentry_chassis_constants.hpp"
#include "aruwsrc/robot/sentry/sentry_chassis_world_yaw_observer.hpp"
#include "aruwsrc/robot/sentry/sentry_control_operator_interface.hpp"
#include "aruwsrc/robot/sentry/sentry_imu_calibrate_command.hpp"
#include "aruwsrc/robot/sentry/sentry_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_launcher_constants.hpp"
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
using namespace tap::control::setpoint;

using namespace aruwsrc::agitator;
using namespace aruwsrc::sentry;
using namespace aruwsrc::control::agitator;
using namespace aruwsrc::sentry::chassis;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::control::sentry;
using namespace aruwsrc::control::turret::sentry;
using namespace aruwsrc::control::turret::algorithms;
using namespace aruwsrc::virtualMCB;
using namespace aruwsrc::control;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace sentry_control
{
aruwsrc::virtualMCB::VirtualDoubleDjiMotor turretMajorYawMotor(
    drivers(),
    &drivers()->chassisMcbLite,
    tap::motor::MOTOR5,
    tap::motor::MOTOR6,
    turretMajor::CAN_BUS_MOTOR_1,
    turretMajor::CAN_BUS_MOTOR_1,
    false,
    false,
    "Major Yaw Turret 1",
    "Major Yaw Turret 2");

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
        "Left Minor Yaw Turret"),

    .pitchMotor = tap::motor::DjiMotor(
        drivers(),
        turretLeft::PITCH_MOTOR_ID,
        turretLeft::CAN_BUS_MOTORS,
        true,
        "Left Minor Pitch Turret"),

    .yawMotorConfig = turretLeft::YAW_MOTOR_CONFIG,
    .pitchMotorConfig = turretLeft::PITCH_MOTOR_CONFIG

};

TurretMinorMotors turretRightMotors{
    .yawMotor = tap::motor::DjiMotor(
        drivers(),
        turretRight::YAW_MOTOR_ID,
        turretRight::CAN_BUS_MOTORS,
        false,
        "Right Minor Yaw Turret"),

    .pitchMotor = tap::motor::DjiMotor(
        drivers(),
        turretRight::PITCH_MOTOR_ID,
        turretRight::CAN_BUS_MOTORS,
        false,
        "Right Minor Pitch Turret"),

    .yawMotorConfig = turretRight::YAW_MOTOR_CONFIG,
    .pitchMotorConfig = turretRight::PITCH_MOTOR_CONFIG

};

inline aruwsrc::can::TurretMCBCanComm &getTurretMCBCanComm()
{
    return drivers()->turretMCBCanCommBus1;
}

/* define subsystems --------------------------------------------------------*/
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

struct TurretMinorWorldControllers
{
    WorldFramePitchTurretImuCascadePidTurretController pitchController;
    WorldFrameYawTurretImuCascadePidTurretController yawController;
};

// @todo surely there's a better way to construct this
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

VirtualDjiMotor rightFrontDriveMotor(
    drivers(),
    MOTOR3,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->chassisMcbLite),
    rightFrontSwerveConfig.driveMotorInverted,
    "Right Front Swerve Drive Motor");

VirtualDjiMotor rightFrontAzimuthMotor(
    drivers(),
    MOTOR7,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->chassisMcbLite),
    rightFrontSwerveConfig.azimuthMotorInverted,
    "Right Front Swerve Azimuth Motor");

VirtualDjiMotor leftBackDriveMotor(
    drivers(),
    MOTOR4,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->chassisMcbLite),
    leftBackSwerveConfig.driveMotorInverted,
    "Left Back Swerve Drive Motor");

VirtualDjiMotor leftBackAzimuthMotor(
    drivers(),
    MOTOR8,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->chassisMcbLite),
    leftBackSwerveConfig.azimuthMotorInverted,
    "Left Back Swerve Azimuth Motor");

// This is the one facing parallel to the frame
VirtualDjiMotor leftOmni(
    drivers(),
    MOTOR1,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->chassisMcbLite),
    false,
    "Right Omni Dead Wheel");

// This is the one sticking out towards the frame
VirtualDjiMotor rightOmni(
    drivers(),
    MOTOR2,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->chassisMcbLite),
    false,
    "Left Omni Dead Wheel");

// these four swerve modules will later be passed into SwerveChassisSubsystem
aruwsrc::chassis::SwerveModule rightFrontSwerveModule(
    rightFrontDriveMotor,
    rightFrontAzimuthMotor,
    rightFrontSwerveConfig);

aruwsrc::chassis::SwerveModule leftBackSwerveModule(
    leftBackDriveMotor,
    leftBackAzimuthMotor,
    leftBackSwerveConfig);

aruwsrc::virtualMCB::VirtualCurrentSensor currentSensor(
    {&drivers()->chassisMcbLite.analog,
     aruwsrc::chassis::CURRENT_SENSOR_PIN,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_MV_PER_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_ZERO_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA});

aruwsrc::chassis::HalfSwerveChassisSubsystem chassis(
    drivers(),
    &currentSensor,
    &rightFrontSwerveModule,
    &leftBackSwerveModule,
    CENTER_TO_WHEELBASE_RADIUS,
    HALF_SWERVE_FORWARD_MATRIX);

SentryKFOdometry2DSubsystem chassisOdometry(
    *drivers(),
    chassis,
    chassisYawObserver,
    drivers()->chassisMcbLite.imu,
    INITIAL_CHASSIS_POSITION_X,
    INITIAL_CHASSIS_POSITION_Y);

SentryTransforms transformer(
    chassisOdometry,
    turretMajor,
    turretLeft,
    turretRight,
    {.turretMinorOffset = TURRET_MINOR_OFFSET});

SentryTransformSubystem transformerSubsystem(*drivers(), transformer);

SentryTransformAdapter transformAdapter(transformer);

tap::algorithms::SmoothPid turretMajorYawPosPid(
    turretMajor::worldFrameCascadeController::YAW_POS_PID_CONFIG);
tap::algorithms::SmoothPid turretMajorYawVelPid(
    turretMajor::worldFrameCascadeController::YAW_VEL_PID_CONFIG);

algorithms::TurretMajorWorldFrameController turretMajorWorldYawController(  // @todo rename
    transformer.getWorldToChassis(),
    chassis,
    turretMajor.getMutableMotor(),
    turretLeft,
    turretRight,
    turretMajorYawPosPid,
    turretMajorYawVelPid,
    turretMajor::MAX_VEL_ERROR_INPUT,
    turretMajor::TURRET_MINOR_TORQUE_RATIO,
    turretMajor::FEEDFORWARD_GAIN);

// Friction Wheels
aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    turretLeftFrictionWheels(
        drivers(),
        aruwsrc::robot::sentry::launcher::LEFT_MOTOR_ID_TURRETLEFT,
        aruwsrc::robot::sentry::launcher::RIGHT_MOTOR_ID_TURRETLEFT,
        turretLeft::CAN_BUS_MOTORS,
        &getTurretMCBCanComm(),
        turretLeft::barrelID);

aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    turretRightFrictionWheels(
        drivers(),
        aruwsrc::robot::sentry::launcher::LEFT_MOTOR_ID_TURRETRIGHT,
        aruwsrc::robot::sentry::launcher::RIGHT_MOTOR_ID_TURRETRIGHT,
        turretRight::CAN_BUS_MOTORS,
        &getTurretMCBCanComm(),
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

/* define commands ----------------------------------------------------------*/
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
    turretMajorWorldYawController,
    chassis,
    chassisYawObserver,
    chassisOdometry,
    drivers()->turretMajorMcbLite,
    drivers()->chassisMcbLite);

// Left
aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    leftFrictionWheels(
        drivers(),
        tap::motor::MOTOR2,
        tap::motor::MOTOR1,
        tap::can::CanBus::CAN_BUS2,
        &drivers()->turretMCBCanCommBus2,
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_2);

// Right
aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    rightFrictionWheels(
        drivers(),
        tap::motor::MOTOR2,
        tap::motor::MOTOR1,
        tap::can::CanBus::CAN_BUS1,
        &drivers()->turretMCBCanCommBus1,
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

aruwsrc::algorithms::OttoBallisticsSolver turretLeftSolver(
    drivers()->visionCoprocessor,
    transformAdapter,
    leftFrictionWheels,
    turretLeft::default_launch_speed,
    transformer.getWorldToTurretMajor(),
    turretMajor.getReadOnlyMotor(),
    TURRET_MINOR_OFFSET,
    turretLeft.getTurretID());

SentryTurretCVCommand::TurretConfig turretLeftCVConfig(
    turretLeft,
    turretLeftWorldControllers.yawController,
    turretLeftWorldControllers.pitchController,
    turretLeftSolver);

aruwsrc::algorithms::OttoBallisticsSolver turretRightSolver(
    drivers()->visionCoprocessor,
    transformAdapter,
    rightFrictionWheels,
    turretRight::default_launch_speed,
    transformer.getWorldToTurretMajor(),
    turretMajor.getReadOnlyMotor(),
    TURRET_MINOR_OFFSET,
    turretRight.getTurretID());

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
aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand turretLeftFrictionWheelSpinCommand(
    drivers(),
    &turretLeftFrictionWheels,
    aruwsrc::robot::sentry::launcher::DESIRED_LAUNCH_SPEED,
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
MoveIntegralCommand turretLeftRotateAgitator(turretLeftAgitator, constants::AGITATOR_ROTATE_CONFIG);
UnjamIntegralCommand turretLeftUnjamAgitator(turretLeftAgitator, constants::AGITATOR_UNJAM_CONFIG);
MoveUnjamIntegralComprisedCommand turretLeftRotateAndUnjamAgitator(
    *drivers(),
    turretLeftAgitator,
    turretLeftRotateAgitator,
    turretLeftUnjamAgitator);

// RIGHT shooting ======================

// spin friction wheels commands
aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand turretRightFrictionWheelSpinCommand(
    drivers(),
    &turretRightFrictionWheels,
    aruwsrc::robot::sentry::launcher::DESIRED_LAUNCH_SPEED,
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
MoveIntegralCommand turretRightRotateAgitator(
    turretRightAgitator,
    constants::AGITATOR_ROTATE_CONFIG);
UnjamIntegralCommand turretRightUnjamAgitator(
    turretRightAgitator,
    constants::AGITATOR_UNJAM_CONFIG);
MoveUnjamIntegralComprisedCommand turretRightRotateAndUnjamAgitator(
    *drivers(),
    turretRightAgitator,
    turretRightRotateAgitator,
    turretRightUnjamAgitator);

/* define command mappings --------------------------------------------------*/
// HoldCommandMapping leftDownRightUp(
//     drivers(),
//     {&imuCalibrateCommand},
//     RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::UP));

HoldCommandMapping leftDown(
    drivers(),
    {&imuCalibrateCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));

// HoldCommandMapping leftMidRightDown(
//     drivers(),
//     {&majorManualCommand, &turretLeftManualCommand, &turretRightManualCommand},
//     RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::DOWN));

HoldCommandMapping leftMid(
    drivers(),
    {&majorManualCommand, &turretLeftManualCommand, &turretRightManualCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

// HoldCommandMapping leftMidRightMid(
//     drivers(),
//     {&chassisDriveCommand},
//     RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::MID));

// HoldCommandMapping shoot(
//     drivers(),
//     {&turretLeftFrictionWheelSpinCommand, &turretRightFrictionWheelSpinCommand},
//     RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::DOWN));

// HoldRepeatCommandMapping leftUpRightDown(
//     drivers(),
//     {&turretLeftRotateAndUnjamAgitator,
//      &turretRightRotateAndUnjamAgitator,
//      &turretLeftFrictionWheelSpinCommand,
//      &turretRightFrictionWheelSpinCommand},
//     RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::DOWN),
//     true);

HoldCommandMapping rightUp(
    drivers(),
    {
        &turretLeftRotateAndUnjamAgitator,
        &turretRightRotateAndUnjamAgitator,
    },
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

// HoldCommandMapping leftDownRightDown(
//     drivers(),
//     {&beybladeCommand},
//     RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::DOWN));

// HoldCommandMapping leftUpRightUp(
//     drivers(),
//     {&turretCVCommand},
//     RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::UP));

HoldCommandMapping leftUp(
    drivers(),
    {&turretCVCommand, &turretLeftFrictionWheelSpinCommand, &turretRightFrictionWheelSpinCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());
/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    chassis.initialize();
    turretLeft.initialize();
    turretRight.initialize();
    turretMajor.initialize();
    chassisOdometry.initialize();
    transformerSubsystem.initialize();

    rightOmni.initialize();
    leftOmni.initialize();

    turretLeftFrictionWheels.initialize();
    turretRightFrictionWheels.initialize();

    turretLeftAgitator.initialize();
    turretRightAgitator.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerSentrySubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&turretMajor);
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&turretLeft);
    drivers->commandScheduler.registerSubsystem(&turretRight);
    drivers->commandScheduler.registerSubsystem(&chassisOdometry);
    drivers->commandScheduler.registerSubsystem(&transformerSubsystem);

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
}

/* add any starting commands to the scheduler here --------------------------*/
void startSentryCommands(Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&imuCalibrateCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerSentryIoMappings(Drivers *drivers)
{
    // left mid: manual
    drivers->commandMapper.addMap(&leftMid);
    // down: imu calibrate
    drivers->commandMapper.addMap(&leftDown);
    // up: cv
    drivers->commandMapper.addMap(&leftUp);
    // right down: no shoot (no mapping)
    // right up : shoot
    drivers->commandMapper.addMap(&rightUp);

    // drivers->commandMapper.addMap(&leftMidRightDown);  // turret manual control
    // drivers->commandMapper.addMap(&leftMidRightDown);   // turret manual control
    // drivers->commandMapper.addMap(&leftDownRightUp);  // imu calibrate command
    // drivers->commandMapper.addMap(&leftMidRightMid);    // chassis drive
    // drivers->commandMapper.addMap(&leftDownRightDown);  // beyblade
    // drivers->commandMapper.addMap(&leftUpRightUp);  // cv
    // drivers->commandMapper.addMap(&leftMidRightDown);   // turret manual control
    // drivers->commandMapper.addMap(&leftDownRightUp);    // imu calibrate command
    // drivers->commandMapper.addMap(&leftMidRightMid);    // chassis drive
    // drivers->commandMapper.addMap(&leftDownRightDown);  // beyblade

    // drivers->commandMapper.addMap(&leftUpRightUp);  // Agitators
    // drivers->commandMapper.addMap(&shoot);          // Shoot
    // drivers->commandMapper.addMap(&leftUpRightDown);  // Shoot
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

#ifndef PLATFORM_HOSTED
imu::ImuCalibrateCommand *getImuCalibrateCommand() { return &sentry_control::imuCalibrateCommand; }
#endif

#endif
