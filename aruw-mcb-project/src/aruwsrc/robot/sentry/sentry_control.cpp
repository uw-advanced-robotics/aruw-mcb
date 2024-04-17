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

#if defined(TARGET_SENTRY_BEEHIVE)
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/double_dji_motor.hpp"

#include "aruwsrc/communication/mcb-lite/motor/virtual_dji_motor.hpp"
#include "aruwsrc/communication/mcb-lite/virtual_current_sensor.hpp"
#include "aruwsrc/control/agitator/constants/agitator_constants.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "aruwsrc/control/chassis/new_sentry/sentry_manual_drive_command.hpp"
#include "aruwsrc/control/chassis/swerve_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/swerve_module.hpp"
#include "aruwsrc/control/chassis/swerve_module_config.hpp"
#include "aruwsrc/control/launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/yaw_turret_subsystem.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/sentry/sentry_beyblade_command.hpp"
#include "aruwsrc/robot/sentry/sentry_chassis_constants.hpp"
#include "aruwsrc/robot/sentry/sentry_chassis_world_yaw_observer.hpp"
#include "aruwsrc/robot/sentry/sentry_control_operator_interface.hpp"
#include "aruwsrc/robot/sentry/sentry_imu_calibrate_command.hpp"
#include "aruwsrc/robot/sentry/sentry_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_launcher_constants.hpp"
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
tap::motor::DoubleDjiMotor turretMajorYawMotor(
    drivers(),
    tap::motor::MOTOR7,
    tap::motor::MOTOR7,
    turretMajor::CAN_BUS_MOTOR_1,
    turretMajor::CAN_BUS_MOTOR_2,
    true,
    true,
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

SentryChassisWorldYawObserver chassisYawObserver(turretMajor, turretLeft, turretRight);

struct TurretMinorControllers
{
    ChassisFramePitchTurretController pitchController;
    ChassisFrameYawTurretController yawController;
};

// @todo make controllers part of subsystem
TurretMinorControllers turretLeftControllers{
    .pitchController = ChassisFramePitchTurretController(
        turretLeft.pitchMotor,
        turretLeft::pidConfigs::PITCH_PID_CONFIG),

    .yawController = ChassisFrameYawTurretController(
        turretLeft.yawMotor,
        turretLeft::pidConfigs::YAW_PID_CONFIG),

};

TurretMinorControllers turretRightControllers{
    .pitchController = ChassisFramePitchTurretController(
        turretRight.pitchMotor,
        turretRight::pidConfigs::PITCH_PID_CONFIG),

    .yawController = ChassisFrameYawTurretController(
        turretRight.yawMotor,
        turretRight::pidConfigs::YAW_PID_CONFIG)

};

VirtualDjiMotor leftFrontDriveMotor(
    drivers(),
    MOTOR2,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->mcbLite),
    leftFrontSwerveConfig.driveMotorInverted,
    "Left Front Swerve Drive Motor");

VirtualDjiMotor leftFrontAzimuthMotor(
    drivers(),
    MOTOR6,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->mcbLite),
    leftFrontSwerveConfig.azimuthMotorInverted,
    "Left Front Swerve Azimuth Motor");

VirtualDjiMotor rightFrontDriveMotor(
    drivers(),
    MOTOR1,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->mcbLite),
    rightFrontSwerveConfig.driveMotorInverted,
    "Right Front Swerve Drive Motor");

VirtualDjiMotor rightFrontAzimuthMotor(
    drivers(),
    MOTOR5,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->mcbLite),
    rightFrontSwerveConfig.azimuthMotorInverted,
    "Right Front Swerve Azimuth Motor");

VirtualDjiMotor leftBackDriveMotor(
    drivers(),
    MOTOR3,
    tap::can::CanBus::CAN_BUS2,
    &(drivers()->mcbLite),
    leftBackSwerveConfig.driveMotorInverted,
    "Left Back Swerve Drive Motor");

VirtualDjiMotor leftBackAzimuthMotor(
    drivers(),
    MOTOR7,
    tap::can::CanBus::CAN_BUS2,
    &(drivers()->mcbLite),
    leftBackSwerveConfig.azimuthMotorInverted,
    "Left Back Swerve Azimuth Motor");

VirtualDjiMotor rightBackDriveMotor(
    drivers(),
    MOTOR4,
    tap::can::CanBus::CAN_BUS2,
    &(drivers()->mcbLite),
    rightBackSwerveConfig.driveMotorInverted,
    "Right Back Swerve Drive Motor");

VirtualDjiMotor rightBackAzimuthMotor(
    drivers(),
    MOTOR8,
    tap::can::CanBus::CAN_BUS2,
    &(drivers()->mcbLite),
    rightBackSwerveConfig.azimuthMotorInverted,
    "Right Back Swerve Azimuth Motor");

// these four swerve modules will later be passed into SwerveChassisSubsystem
aruwsrc::chassis::SwerveModule leftFrontSwerveModule(
    leftFrontDriveMotor,
    leftFrontAzimuthMotor,
    leftFrontSwerveConfig);

aruwsrc::chassis::SwerveModule rightFrontSwerveModule(
    rightFrontDriveMotor,
    rightFrontAzimuthMotor,
    rightFrontSwerveConfig);

aruwsrc::chassis::SwerveModule leftBackSwerveModule(
    leftBackDriveMotor,
    leftBackAzimuthMotor,
    leftBackSwerveConfig);

aruwsrc::chassis::SwerveModule rightBackSwerveModule(
    rightBackDriveMotor,
    rightBackAzimuthMotor,
    rightBackSwerveConfig);

aruwsrc::virtualMCB::VirtualCurrentSensor currentSensor(
    {&drivers()->mcbLite.analog,
     aruwsrc::chassis::CURRENT_SENSOR_PIN,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_MV_PER_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_ZERO_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA});

aruwsrc::chassis::SwerveChassisSubsystem chassis(
    drivers(),
    &currentSensor,
    &leftFrontSwerveModule,
    &rightFrontSwerveModule,
    &leftBackSwerveModule,
    &rightBackSwerveModule,
    SWERVE_FORWARD_MATRIX);

SentryKFOdometry2DSubsystem chassisOdometry(
    *drivers(),
    chassis,
    chassisYawObserver,
    drivers()->mcbLite.imu,
    INITIAL_CHASSIS_POSITION_X,
    INITIAL_CHASSIS_POSITION_Y);

SentryTransforms transformer(
    chassisOdometry,
    turretMajor,
    turretLeft,
    turretRight,
    {.turretMinorOffset = TURRET_MINOR_OFFSET});

SentryTransformSubystem transformerSubsystem(*drivers(), transformer);

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
    turretMajor::TURRET_MINOR_TORQUE_RATIO);

// Friction Wheels
aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    frictionWheelsTurretLeft(
        drivers(),
        aruwsrc::robot::sentry::launcher::LEFT_MOTOR_ID_TURRETLEFT,
        aruwsrc::robot::sentry::launcher::RIGHT_MOTOR_ID_TURRETLEFT,
        turretLeft::CAN_BUS_MOTORS,
        &getTurretMCBCanComm(),
        turretLeft::barrelID);

aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    frictionWheelsTurretRight(
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
    turretLeftControllers.yawController,
    turretLeftControllers.pitchController,
    MINOR_USER_YAW_INPUT_SCALAR,
    MINOR_USER_PITCH_INPUT_SCALAR);

TurretMinorSentryControlCommand turretRightManualCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    turretRight,
    turretRightControllers.yawController,
    turretRightControllers.pitchController,
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
            turretLeft,
            turretLeftControllers.yawController,
            turretLeftControllers.pitchController,
            true,
        },
        {
            &drivers()->turretMCBCanCommBus1,
            turretRight,
            turretRightControllers.yawController,
            turretRightControllers.pitchController,
            true,
        },
    },
    turretMajor,
    turretMajorWorldYawController,
    chassis,
    chassisYawObserver,
    chassisOdometry);

// LEFT shooting ======================

// spin friction wheels commands
aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand turretLeftFrictionWheelSpinCommand(
    drivers(),
    &frictionWheelsTurretLeft,
    aruwsrc::robot::sentry::launcher::DESIRED_LAUNCH_SPEED,
    false,
    turretLeft::barrelID);

aruwsrc::control::launcher::
    FrictionWheelSpinRefLimitedCommand stopTurretLeftFrictionWheelSpinCommand(
        drivers(),
        &frictionWheelsTurretLeft,
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
    &frictionWheelsTurretRight,
    aruwsrc::robot::sentry::launcher::DESIRED_LAUNCH_SPEED,
    false,
    turretRight::barrelID);

aruwsrc::control::launcher::
    FrictionWheelSpinRefLimitedCommand stopTurretRightFrictionWheelSpinCommand(
        drivers(),
        &frictionWheelsTurretRight,
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
HoldCommandMapping leftDownRightUp(
    drivers(),
    {&imuCalibrateCommand},
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::UP));

HoldCommandMapping leftMidRightDown(
    drivers(),
    {&majorManualCommand, &turretLeftManualCommand, &turretRightManualCommand},
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::DOWN));

HoldCommandMapping leftMidRightMid(
    drivers(),
    {&chassisDriveCommand},
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::MID));

HoldRepeatCommandMapping leftUpRightUp(
    drivers(),
    {&turretLeftRotateAndUnjamAgitator, &turretRightRotateAndUnjamAgitator},
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::UP),
    true);

HoldCommandMapping shoot(
    drivers(),
    {&turretLeftFrictionWheelSpinCommand, &turretRightFrictionWheelSpinCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

HoldCommandMapping leftDownRightDown(
    drivers(),
    {&beybladeCommand},
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::DOWN));

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

    // turret
    frictionWheelsTurretLeft.initialize();
    turretLeftAgitator.initialize();
    frictionWheelsTurretRight.initialize();
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
    drivers->commandScheduler.registerSubsystem(&frictionWheelsTurretLeft);
    drivers->commandScheduler.registerSubsystem(&frictionWheelsTurretRight);

    drivers->commandScheduler.registerSubsystem(&turretLeftAgitator);
    drivers->commandScheduler.registerSubsystem(&turretRightAgitator);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSentryCommands(Drivers *)
{
    //chassis.setDefaultCommand(&chassisDriveCommand);
    turretMajor.setDefaultCommand(&majorManualCommand);
    turretLeft.setDefaultCommand(&turretLeftManualCommand);
    turretRight.setDefaultCommand(&turretRightManualCommand);

    frictionWheelsTurretLeft.setDefaultCommand(&stopTurretLeftFrictionWheelSpinCommand);
    frictionWheelsTurretRight.setDefaultCommand(&stopTurretRightFrictionWheelSpinCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSentryCommands(Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&imuCalibrateCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerSentryIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(&leftMidRightDown);  // turret manual control
    drivers->commandMapper.addMap(&leftDownRightUp);   // imu calibrate command
    drivers->commandMapper.addMap(&leftMidRightMid);   // chassis drive
    drivers->commandMapper.addMap(&leftDownRightDown);  // beyblade

    drivers->commandMapper.addMap(&leftUpRightUp);  //Agitators
    drivers->commandMapper.addMap(&shoot);  // Shoot
    
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

#endif
