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
#if defined(TARGET_SENTRY_BEEHIVE)

#include "aruwsrc/control/turret/algorithms/world_frame_turret_yaw_controller.hpp"
#include "aruwsrc/robot/sentry/match_running_governor.hpp"

#include "tap/control/command_mapper.hpp"
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/governor/governor_with_fallback_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/motor/double_dji_motor.hpp"

#include "aruwsrc/algorithms/odometry/chassis_kf_odometry.hpp"
#include "aruwsrc/communication/mcb-lite/motor/virtual_dji_motor.hpp"
#include "aruwsrc/communication/mcb-lite/serial_mcb_lite.hpp"
#include "aruwsrc/control/agitator/constants/agitator_constants.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/chassis/swerve_chassis_subsystem.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "aruwsrc/control/agitator/manual_fire_rate_reselection_manager.hpp"
#include "aruwsrc/control/governor/fire_rate_limit_governor.hpp"
#include "aruwsrc/control/governor/friction_wheels_on_governor.hpp"
#include "aruwsrc/control/governor/heat_limit_governor.hpp"
// #include "aruwsrc/control/governor/pause_command_governor.hpp"
// #include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "aruwsrc/control/governor/ref_system_projectile_launched_governor.hpp"
#include "aruwsrc/control/imu/sentry_imu_calibrate_command.hpp"
#include "aruwsrc/control/launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/governor/pause_command_governor.hpp"
#include "aruwsrc/drivers_singleton.hpp"
// #include "aruwsrc/robot/sentry/sentry_otto_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/control/chassis/new_sentry/sentry_manual_drive_command.hpp"
#include "aruwsrc/control/chassis/swerve_module.hpp"
#include "aruwsrc/control/turret/sentry/turret_major_sentry_control_command.hpp"
#include "aruwsrc/control/turret/sentry/turret_minor_sentry_control_command.hpp"
#include "aruwsrc/robot/sentry/sentry_beehive_chassis_constants.hpp"
#include "aruwsrc/robot/sentry/sentry_chassis_world_yaw_observer.hpp"
#include "aruwsrc/robot/sentry/sentry_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_cv_command.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_major_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_minor_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_strategy_request_handler.hpp"
#include "aruwsrc/robot/sentry/sentry_request_message_types.hpp"
#include "aruwsrc/robot/sentry/sentry_beehive_launcher_constants.hpp"
#include "aruwsrc/robot/sentry/sentry_response_transmitter.hpp"
#include "aruwsrc/robot/sentry/sentry_response_message_types.hpp"

#include "sentry_transform_constants.hpp"
#include "sentry_transforms.hpp"
#include "sentry_auto_aim_launch_timer.hpp"
#include "sentry_minor_cv_on_target_governor.hpp"
#include "sentry_beyblade_command.hpp"
#include "aruwsrc/control/auto-aim/auto_aim_fire_rate_reselection_manager.hpp"
#include "aruwsrc/control/chassis/autonav/auto_nav_command.hpp"
#include "aruwsrc/control/chassis/autonav/auto_nav_beyblade_command.hpp"
#include "aruwsrc/robot/sentry/sentry_aruco_reset_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_response_subsystem.hpp"

using namespace tap::control::governor;
using namespace tap::control::setpoint;
using namespace aruwsrc::agitator;
using namespace tap::gpio;
using namespace aruwsrc::control;
using namespace tap::control;
using namespace tap::communication::serial;
using namespace tap::motor;
using namespace aruwsrc::control::governor;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::control::agitator;
using namespace aruwsrc::control::launcher;
using namespace aruwsrc::algorithms::odometry;
using namespace aruwsrc::algorithms;
using namespace aruwsrc::algorithms::odometry;
using namespace tap::communication::serial;
using namespace aruwsrc::sentry;
using namespace aruwsrc::virtualMCB;

namespace sentry_control
{

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

aruwsrc::communication::serial::SentryStrategyRequestHandler sentryStrategyRequestHandler(drivers());
aruwsrc::communication::serial::SentryResponseTransmitter sentryResponseTransmitter(drivers());
aruwsrc::communication::serial::SentryResponseSubsystem sentryResponseSubsystem(drivers(), sentryResponseTransmitter);

/* define swerve motors --------------------------------------------------------*/

VirtualDjiMotor leftFrontDriveMotor(
    drivers(),
    MOTOR2,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->mcbLite),
    aruwsrc::sentry::chassis::leftFrontSwerveConfig.driveMotorInverted,
    "Left Front Swerve Drive Motor");

VirtualDjiMotor leftFrontAzimuthMotor(
    drivers(),
    MOTOR6,
    // MOTOR5,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->mcbLite),
    aruwsrc::sentry::chassis::leftFrontSwerveConfig.azimuthMotorInverted,
    "Left Front Swerve Azimuth Motor");

VirtualDjiMotor rightFrontDriveMotor(
    drivers(),
    MOTOR1,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->mcbLite),
    aruwsrc::sentry::chassis::rightFrontSwerveConfig.driveMotorInverted,  // TODO: BRUHHH
    "Right Front Swerve Drive Motor");

VirtualDjiMotor rightFrontAzimuthMotor(
    drivers(),
    MOTOR5,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->mcbLite),
    aruwsrc::sentry::chassis::rightFrontSwerveConfig.azimuthMotorInverted,
    "Right Front Swerve Azimuth Motor");

VirtualDjiMotor leftBackDriveMotor(
    drivers(),
    MOTOR3,
    tap::can::CanBus::CAN_BUS2,
    &(drivers()->mcbLite),
    aruwsrc::sentry::chassis::leftBackSwerveConfig.driveMotorInverted,
    "Left Back Swerve Drive Motor");

VirtualDjiMotor leftBackAzimuthMotor(
    drivers(),
    MOTOR7,
    tap::can::CanBus::CAN_BUS2,
    &(drivers()->mcbLite),
    aruwsrc::sentry::chassis::leftBackSwerveConfig.azimuthMotorInverted,
    "Left Back Swerve Azimuth Motor");

VirtualDjiMotor rightBackDriveMotor(
    drivers(),
    MOTOR4,
    tap::can::CanBus::CAN_BUS2,
    &(drivers()->mcbLite),
    aruwsrc::sentry::chassis::rightBackSwerveConfig.driveMotorInverted,
    "Right Back Swerve Drive Motor");

VirtualDjiMotor rightBackAzimuthMotor(
    drivers(),
    MOTOR8,
    tap::can::CanBus::CAN_BUS2,
    &(drivers()->mcbLite),
    aruwsrc::sentry::chassis::rightBackSwerveConfig.azimuthMotorInverted,
    "Right Back Swerve Azimuth Motor");

// these four swerve modules will later be passed into SwerveChassisSubsystem
aruwsrc::chassis::SwerveModule leftFrontSwerveModule(
    leftFrontDriveMotor,
    leftFrontAzimuthMotor,
    aruwsrc::sentry::chassis::leftFrontSwerveConfig);

aruwsrc::chassis::SwerveModule rightFrontSwerveModule(
    rightFrontDriveMotor,
    rightFrontAzimuthMotor,
    aruwsrc::sentry::chassis::rightFrontSwerveConfig);

aruwsrc::chassis::SwerveModule leftBackSwerveModule(
    leftBackDriveMotor,
    leftBackAzimuthMotor,
    aruwsrc::sentry::chassis::leftBackSwerveConfig);

aruwsrc::chassis::SwerveModule rightBackSwerveModule(
    rightBackDriveMotor,
    rightBackAzimuthMotor,
    aruwsrc::sentry::chassis::rightBackSwerveConfig);

/* define turret motors --------------------------------------------------------*/

tap::motor::DoubleDjiMotor turretMajorYawMotor(
    drivers(),
    MOTOR7,
    MOTOR7,
    turretMajor::CAN_BUS_MOTOR_1,
    turretMajor::CAN_BUS_MOTOR_2,
    true,
    true,
    "Major Yaw Turret 1",
    "Major Yaw Turret 2");

tap::motor::DjiMotor turretMinorGirlbossYawMotor(
    drivers(),
    MOTOR6,
    girlboss::CAN_BUS_MOTORS,
    false,
    "Minor girlboss Yaw Turret");

tap::motor::DjiMotor turretMinorGirlbossPitchMotor(
    drivers(),
    MOTOR5,
    girlboss::CAN_BUS_MOTORS,
    true,
    "Minor girlboss Pitch Turret");

tap::motor::DjiMotor turretMinorMalewifeYawMotor(
    drivers(),
    MOTOR6,
    malewife::CAN_BUS_MOTORS,
    false,
    "Minor malewife Yaw Turret");

tap::motor::DjiMotor turretMinorMalewifePitchMotor(
    drivers(),
    MOTOR5,
    malewife::CAN_BUS_MOTORS,
    false,
    "Minor malewife Pitch Turret");

/* turret can ---------------------------------------------------------------*/

// @todo errrr
inline aruwsrc::can::TurretMCBCanComm &getTurretMCBCanComm()
{
    return drivers()->turretMCBCanCommBus1;
}

/* define subsystems --------------------------------------------------------*/

// Chassis
aruwsrc::chassis::SwerveChassisSubsystem sentryDrive(
    drivers(),
    &drivers()->mcbLite.currentSensor,
    &leftFrontSwerveModule,
    &rightFrontSwerveModule,
    &leftBackSwerveModule,
    &rightBackSwerveModule,
    aruwsrc::sentry::chassis::SWERVE_FORWARD_MATRIX);

// Turret Major
SentryTurretMajorSubsystem turretMajor(
    drivers(),
    &turretMajorYawMotor,
    aruwsrc::control::turret::turretMajor::YAW_MOTOR_CONFIG);

// Turret Minors ---------------------------------------------------------
SentryTurretMinorSubsystem turretMinorGirlboss(
    drivers(),
    &turretMinorGirlbossPitchMotor,
    &turretMinorGirlbossYawMotor,
    aruwsrc::control::turret::girlboss::PITCH_MOTOR_CONFIG,
    aruwsrc::control::turret::girlboss::YAW_MOTOR_CONFIG,
    &drivers()->turretMCBCanCommBus2,
    girlboss::turretID);

SentryTurretMinorSubsystem turretMinorMalewife(
    drivers(),
    &turretMinorMalewifePitchMotor,
    &turretMinorMalewifeYawMotor,
    aruwsrc::control::turret::malewife::PITCH_MOTOR_CONFIG,
    aruwsrc::control::turret::malewife::YAW_MOTOR_CONFIG,
    &drivers()->turretMCBCanCommBus1,
    malewife::turretID);

// Friction wheels ---------------------------------------------------------------------------

aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    frictionWheelsGirlboss(
        drivers(),
        aruwsrc::robot::sentry::launcher::LEFT_MOTOR_ID_GIRLBOSS,
        aruwsrc::robot::sentry::launcher::RIGHT_MOTOR_ID_GIRLBOSS,
        girlboss::CAN_BUS_MOTORS,
        &getTurretMCBCanComm(),
        girlboss::barrelID);

aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    frictionWheelsMalewife(
        drivers(),
        aruwsrc::robot::sentry::launcher::LEFT_MOTOR_ID_MALEWIFE,
        aruwsrc::robot::sentry::launcher::RIGHT_MOTOR_ID_MALEWIFE,
        malewife::CAN_BUS_MOTORS,
        &getTurretMCBCanComm(),
        malewife::barrelID);  // @todo idk what they actually are

// Agitators
VelocityAgitatorSubsystem girlbossAgitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::girlboss::AGITATOR_CONFIG);

VelocityAgitatorSubsystem malewifeAgitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::malewife::AGITATOR_CONFIG);

// Odometry ----------------------------------------------------------------------------------

SentryChassisWorldYawObserver sentryChassisWorldYawObserver(
    turretMajor,
    turretMinorGirlboss,
    turretMinorMalewife);

// On other robots, the turret IMU defines the world frame on initialization, apparently because
// type C's have better IMUs than type A's We need to decide which IMU to use Also subsystem for
// odometry is cringe (?) so we're not using it
SentryKFOdometry2DSubsystem odometrySubsystem(
    *drivers(),
    sentryDrive,
    sentryChassisWorldYawObserver,
    drivers()->mcbLite.imu,
    modm::Location2D<float>(0., 0., M_PI * 0.75),
    0.5f, 0.5f);  // TODO: this


// Transforms --------------------------------------------------------------------------------

SentryTransformsSubsystem sentryTransforms(
    *drivers(),
    odometrySubsystem,
    turretMajor,
    turretMinorGirlboss,
    turretMinorMalewife,
    SENTRY_TRANSFORM_CONFIG);
// Turret controllers -------------------------------------------------------

// @todo make controllers part of subsystem
algorithms::ChassisFramePitchTurretController girlbossPitchController(
    turretMinorGirlboss.pitchMotor,
    major_rel::girlboss::PITCH_PID_CONFIG);

algorithms::ChassisFramePitchTurretController malewifePitchController(
    turretMinorMalewife.pitchMotor,
    major_rel::malewife::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController girlbossYawController(
    turretMinorGirlboss.yawMotor,
    major_rel::girlboss::YAW_PID_CONFIG);

algorithms::ChassisFrameYawTurretController malewifeYawController(
    turretMinorMalewife.yawMotor,
    major_rel::malewife::YAW_PID_CONFIG);

// @todo interesting circular dependency issue since transforms required by controller but subsystem required by transforms
// Because there is no thing for the turret major, we need to instantiate
// a yaw controller for the turret major ourselves
tap::algorithms::SmoothPid turretMajorYawPosPid(turretMajor::YAW_POS_PID_CONFIG);
tap::algorithms::SmoothPid turretMajorYawVelPid(turretMajor::YAW_VEL_PID_CONFIG);

algorithms::WorldFrameTurretYawCascadePIDController turretMajorYawController(  // @todo rename
    sentryTransforms.getWorldToChassis(),
    sentryDrive,
    turretMajor.yawMotor,
    turretMinorGirlboss,
    turretMinorMalewife,
    turretMajorYawPosPid,
    turretMajorYawVelPid,
    aruwsrc::control::turret::turretMajor::MAX_VEL_ERROR_INPUT,
    aruwsrc::control::turret::turretMajor::TURRET_MINOR_TORQUE_RATIO);


// aruco reset


SentryArucoResetSubsystem arucoResetSubsystem(
    *drivers(),
    sentryChassisWorldYawObserver,
    odometrySubsystem,
    drivers()->visionCoprocessor,
    sentryTransforms,
    turretMajorYawController);


// Otto ballistics solver --------------------------------------------------------------------

OttoBallisticsSolver<TurretMinorGirlbossFrame> girlbossBallisticsSolver(
    odometrySubsystem,
    turretMajor,
    sentryTransforms.getWorldToTurretGirlboss(),
    sentryTransforms,
    frictionWheelsGirlboss,
    girlboss::default_launch_speed,
    -girlboss::majorToTurretR);  // girlboss is on the left

SentryAutoAimLaunchTimer autoAimLaunchTimerGirlBoss(
    aruwsrc::robot::sentry::launcher::AGITATOR_TYPICAL_DELAY_MICROSECONDS,
    &drivers()->visionCoprocessor,
    (OttoBallisticsSolver<TurretMinorFrame>*) &girlbossBallisticsSolver,
    girlboss::turretID);

OttoBallisticsSolver<TurretMinorMalewifeFrame> malewifeBallisticsSolver(
    odometrySubsystem,
    turretMajor,
    sentryTransforms.getWorldToTurretMalewife(),
    sentryTransforms,
    frictionWheelsMalewife,
    malewife::default_launch_speed,
    malewife::majorToTurretR);

SentryAutoAimLaunchTimer autoAimLaunchTimerMalewife(
    aruwsrc::robot::sentry::launcher::AGITATOR_TYPICAL_DELAY_MICROSECONDS,
    &drivers()->visionCoprocessor,
    (OttoBallisticsSolver<TurretMinorFrame>*) &malewifeBallisticsSolver,
    malewife::turretID);
// Benjamin rant: what we combined the flywheels, agitator, and turret pitch/yaw motors into a
// single subsystem called Turret? It would have functions like prep-to-shoot, shoot, turn, and
// things like that. What if controllers were mix-ins for susbsystems or something?
// FIXME: Quote Derek: there's an issue to refactor the controller into the subsystem!!

/* define commands ----------------------------------------------------------*/

imu::SentryImuCalibrateCommand imuCalibrateCommand(
    drivers(),
    {
        {
            &drivers()->turretMCBCanCommBus2,
            &turretMinorGirlboss,
            &girlbossYawController,
            &girlbossPitchController,
            true,
        },
        {
            &drivers()->turretMCBCanCommBus1,
            &turretMinorMalewife,
            &malewifeYawController,
            &malewifePitchController,
            true,
        },
    },
    &turretMajor,
    &turretMajorYawController,
    &sentryDrive,
    sentryChassisWorldYawObserver,
    odometrySubsystem);

// Chassis drive manual
aruwsrc::control::sentry::SentryManualDriveCommand chassisDriveCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &sentryDrive);


// Chassis beyblade
aruwsrc::sentry::SentryBeybladeCommand beybladeCommand(
    drivers(),
    &sentryDrive,
    &turretMajor.yawMotor,
    drivers()->controlOperatorInterface,
    sentryTransforms.getWorldToChassis(),
    aruwsrc::sentry::chassis::beybladeConfig
);

// Turret major control manual
aruwsrc::control::turret::sentry::TurretMajorSentryControlCommand turretMajorControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turretMajor,
    &turretMajorYawController,
    aruwsrc::control::turret::MAJOR_USER_YAW_INPUT_SCALAR);

// Turret minor control manual
aruwsrc::control::turret::sentry::TurretMinorSentryControlCommand turretMinorGirlbossControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    turretMinorGirlboss,
    girlbossYawController,
    girlbossPitchController,
    MINOR_USER_YAW_INPUT_SCALAR,
    MINOR_USER_PITCH_INPUT_SCALAR);

aruwsrc::control::turret::sentry::TurretMinorSentryControlCommand turretMinorMalewifeControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    turretMinorMalewife,
    malewifeYawController,
    malewifePitchController,
    MINOR_USER_YAW_INPUT_SCALAR,
    MINOR_USER_PITCH_INPUT_SCALAR);

// random command
aruwsrc::control::turret::SentryTurretCVCommand sentryTurretCVCommand(
    drivers()->visionCoprocessor,
    turretMajor,
    turretMinorGirlboss,
    turretMinorMalewife,
    turretMajorYawController,  // Create + use CV version??
    girlbossYawController,
    girlbossPitchController,
    malewifeYawController,
    malewifePitchController,
    girlbossBallisticsSolver,
    malewifeBallisticsSolver,
    sentryTransforms);

// aruwsrc::chassis::AutoNavCommand autoNavCommand(
//     *drivers(),
//     sentryDrive,
//     turretMajor.yawMotor,
//     drivers()->visionCoprocessor,
//     odometrySubsystem);

aruwsrc::chassis::AutoNavBeybladeCommand autoNavBeybladeCommand(
    *drivers(),
    sentryDrive,
    turretMajor.yawMotor,
    drivers()->visionCoprocessor,
    odometrySubsystem,
    sentryResponseTransmitter,
    aruwsrc::sentry::chassis::beybladeConfig,
    false);  // @todo this should be in config!


// general shooting ===================================

PauseCommandGovernor holdFireGovernor(10000);
// girlboss shooting ======================
// girlboss shooting ======================
// girlboss shooting ======================
// girlboss shooting ======================
// girlboss shooting ======================
// girlboss shooting ======================

// spin friction wheels commands
aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand girlbossFrictionWheelSpinCommand(
    drivers(),
    &frictionWheelsGirlboss,
    aruwsrc::robot::sentry::launcher::DESIRED_LAUNCH_SPEED,
    false,
    girlboss::barrelID);

aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand stopGirlBossFrictionWheelSpinCommand(
    drivers(),
    &frictionWheelsGirlboss,
    0.0f,
    true,
    girlboss::barrelID);

FrictionWheelsOnGovernor frictionWheelsOnGovernorGirlboss(frictionWheelsGirlboss);

// Agitator commands (girl boss)
MoveIntegralCommand girlbossRotateAgitator(girlbossAgitator, constants::AGITATOR_ROTATE_CONFIG);
UnjamIntegralCommand girlbossUnjamAgitator(girlbossAgitator, constants::AGITATOR_UNJAM_CONFIG);
MoveUnjamIntegralComprisedCommand girlbossRotateAndUnjamAgitator(
    *drivers(),
    girlbossAgitator,
    girlbossRotateAgitator,
    girlbossUnjamAgitator);

RefSystemProjectileLaunchedGovernor refSystemProjectileLaunchedGovernorGirlboss(
    drivers()->refSerial,
    girlboss::barrelID);

AutoAimFireRateReselectionManager fireRateReselectionManagerGirlboss(
    *drivers(),
    drivers()->visionCoprocessor,
    drivers()->commandScheduler,
    sentryTurretCVCommand,
    girlboss::turretID
);

FireRateLimitGovernor fireRateLimitGovernorGirlboss(fireRateReselectionManagerGirlboss);

// rotates agitator with heat limiting applied
HeatLimitGovernor heatLimitGovernorGirlboss(
    *drivers(),
    girlboss::barrelID,
    constants::HEAT_LIMIT_BUFFER);

// rotates agitator when aiming at target and within heat limit
SentryMinorCvOnTargetGovernor cvOnTargetGovernorGirlboss(
    ((tap::Drivers *)(drivers())),
    drivers()->visionCoprocessor,
    sentryTurretCVCommand,
    autoAimLaunchTimerGirlBoss,
    SentryCvOnTargetGovernorMode::ON_TARGET_AND_GATED,
    girlboss::turretID);

MatchRunningGovernor matchRunningGovernor(drivers()->refSerial);

GovernorLimitedCommand<4> girlbossRotateAndUnjamAgitatorWithHeatLimiting(
    {&girlbossAgitator},
    girlbossRotateAndUnjamAgitator,
    {&heatLimitGovernorGirlboss, &refSystemProjectileLaunchedGovernorGirlboss, &frictionWheelsOnGovernorGirlboss, &fireRateLimitGovernorGirlboss});

GovernorLimitedCommand<6> girlbossRotateAndUnjamAgitatorWithCVAndHeatLimiting(
    {&girlbossAgitator},
    girlbossRotateAndUnjamAgitator,
    // {&heatLimitGovernorGirlboss, &refSystemProjectileLaunchedGovernorGirlboss, &frictionWheelsOnGovernorGirlboss, &fireRateLimitGovernorGirlboss, &cvOnTargetGovernorGirlboss});
    {&heatLimitGovernorGirlboss, &refSystemProjectileLaunchedGovernorGirlboss, &frictionWheelsOnGovernorGirlboss, &fireRateLimitGovernorGirlboss, &cvOnTargetGovernorGirlboss, &matchRunningGovernor});

// malewife shooting ======================
// malewife shooting ======================
// malewife shooting ======================
// malewife shooting ======================
// malewife shooting ======================
// malewife shooting ======================
// malewife shooting ======================
// malewife shooting ======================
// malewife shooting ======================

// spin friction wheels commands
aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand malewifeFrictionWheelSpinCommand(
    drivers(),
    &frictionWheelsMalewife,
    aruwsrc::robot::sentry::launcher::DESIRED_LAUNCH_SPEED,
    false,
    malewife::barrelID);

aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand stopMaleWifeFrictionWheelSpinCommand(
    drivers(),
    &frictionWheelsMalewife,
    0.0f,
    true,
    malewife::barrelID);

FrictionWheelsOnGovernor frictionWheelsOnGovernorMalewife(frictionWheelsMalewife);

// Agitator commands (male wife)
MoveIntegralCommand malewifeRotateAgitator(malewifeAgitator, constants::AGITATOR_ROTATE_CONFIG);
UnjamIntegralCommand malewifeUnjamAgitator(malewifeAgitator, constants::AGITATOR_UNJAM_CONFIG);
MoveUnjamIntegralComprisedCommand malewifeRotateAndUnjamAgitator(
    *drivers(),
    malewifeAgitator,
    malewifeRotateAgitator,
    malewifeUnjamAgitator);

RefSystemProjectileLaunchedGovernor refSystemProjectileLaunchedGovernorMalewife(
    drivers()->refSerial,
    malewife::barrelID);

AutoAimFireRateReselectionManager fireRateReselectionManagerMalewife(
    *drivers(),
    drivers()->visionCoprocessor,
    drivers()->commandScheduler,
    sentryTurretCVCommand,
    malewife::turretID
);

FireRateLimitGovernor fireRateLimitGovernorMalewife(fireRateReselectionManagerMalewife);

// rotates agitator with heat limiting applied
HeatLimitGovernor heatLimitGovernorMalewife(
    *drivers(),
    malewife::barrelID,
    constants::HEAT_LIMIT_BUFFER);

// rotates agitator when aiming at target and within heat limit
SentryMinorCvOnTargetGovernor cvOnTargetGovernorMalewife(
    ((tap::Drivers *)(drivers())),
    drivers()->visionCoprocessor,
    sentryTurretCVCommand,
    autoAimLaunchTimerMalewife,
    SentryCvOnTargetGovernorMode::ON_TARGET_AND_GATED,
    malewife::turretID);

// @todo wtf
GovernorLimitedCommand<4> malewifeRotateAndUnjamAgitatorWithHeatLimiting(
    {&malewifeAgitator},
    malewifeRotateAndUnjamAgitator,
    {&heatLimitGovernorMalewife, &refSystemProjectileLaunchedGovernorMalewife, &frictionWheelsOnGovernorMalewife, &fireRateLimitGovernorMalewife});

GovernorLimitedCommand<6> malewifeRotateAndUnjamAgitatorWithCVAndHeatLimiting(
    {&malewifeAgitator},
    malewifeRotateAndUnjamAgitator,
    // {&heatLimitGovernorMalewife, &refSystemProjectileLaunchedGovernorMalewife, &frictionWheelsOnGovernorMalewife, &fireRateLimitGovernorMalewife, &cvOnTargetGovernorMalewife});
    {&heatLimitGovernorMalewife, &refSystemProjectileLaunchedGovernorMalewife, &frictionWheelsOnGovernorMalewife, &fireRateLimitGovernorMalewife, &cvOnTargetGovernorMalewife, &matchRunningGovernor});

// callbacks the sentry runs when receiving a message
void sendNoMotionStrategy() { 
    drivers()->visionCoprocessor.sendMotionStrategyMessage(aruwsrc::communication::serial::SentryRequestType::NONE); 
    sentryResponseTransmitter.queueRequest(aruwsrc::communication::serial::SentryResponseType::NONE);
}
void sendGoToFriendlyBaseStrategy() { 
    drivers()->visionCoprocessor.sendMotionStrategyMessage(aruwsrc::communication::serial::SentryRequestType::GO_TO_FRIENDLY_BASE); 
    sentryResponseTransmitter.queueRequest(aruwsrc::communication::serial::SentryResponseType::GO_TO_FRIENDLY_BASE);
}
void sendGoToEnemyBaseStrategy() { 
    drivers()->visionCoprocessor.sendMotionStrategyMessage(aruwsrc::communication::serial::SentryRequestType::GO_TO_ENEMY_BASE);
    sentryResponseTransmitter.queueRequest(aruwsrc::communication::serial::SentryResponseType::GO_TO_ENEMY_BASE);
}
void sendGoToFriendlySupplierZoneStrategy() { 
    drivers()->visionCoprocessor.sendMotionStrategyMessage(aruwsrc::communication::serial::SentryRequestType::GO_TO_FRIENDLY_SUPPLIER_ZONE);
    sentryResponseTransmitter.queueRequest(aruwsrc::communication::serial::SentryResponseType::GO_TO_FRIENDLY_SUPPLIER_ZONE);
}

void sendGoToEnemySupplierZoneStrategy() {
    drivers()->visionCoprocessor.sendMotionStrategyMessage(aruwsrc::communication::serial::SentryRequestType::GO_TO_ENEMY_SUPPLIER_ZONE);
    sentryResponseTransmitter.queueRequest(aruwsrc::communication::serial::SentryResponseType::GO_TO_ENEMY_SUPPLIER_ZONE);
}

void sendGoToCenterPointStrategy() { 
    drivers()->visionCoprocessor.sendMotionStrategyMessage(aruwsrc::communication::serial::SentryRequestType::GO_TO_CENTER_POINT);
    sentryResponseTransmitter.queueRequest(aruwsrc::communication::serial::SentryResponseType::GO_TO_CENTER_POINT);
}

void holdFireCallback() { 
    holdFireGovernor.initiatePause();
    sentryResponseTransmitter.queueRequest(aruwsrc::communication::serial::SentryResponseType::HOLD_FIRE);
}

void toggleMovementCallback() {
    autoNavBeybladeCommand.toggleMovement();
}

void toggleBeybladeCallback() {
    autoNavBeybladeCommand.toggleBeyblade();
}

/* define command mappings --------------------------------------------------*/

// @todo this whole thing looks scuffed but that's because there are a lot of workarounds for taproot's command mapping system

// Manual Mode (left down)

HoldCommandMapping manualRightSwitchUp(
    drivers(),
    {&imuCalibrateCommand},
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::UP));

// Auto Mode (left mid)

HoldCommandMapping autoRightSwitchDown(
    drivers(),
    {&autoNavBeybladeCommand,
     &turretMajorControlCommand,
     &turretMinorGirlbossControlCommand,
     &turretMinorMalewifeControlCommand},
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::DOWN));

HoldCommandMapping autoRightSwitchMid(
    drivers(),
    {&sentryTurretCVCommand, &chassisDriveCommand},
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::MID));

HoldCommandMapping autoRightSwitchUp(
    drivers(),
    {&autoNavBeybladeCommand, &sentryTurretCVCommand},
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::UP));

// Shoot Mode (left up)

HoldCommandMapping shoot(
    drivers(),
    {&girlbossFrictionWheelSpinCommand, &malewifeFrictionWheelSpinCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

HoldRepeatCommandMapping shootRightSwitchDownAgitator(
    drivers(),
    {&girlbossRotateAndUnjamAgitatorWithHeatLimiting, &malewifeRotateAndUnjamAgitatorWithHeatLimiting},
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::DOWN),
    true);

HoldCommandMapping shootRightSwitchMid(
    drivers(),
    {&sentryTurretCVCommand},
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::MID));

HoldCommandMapping shootRightSwitchUp(
    drivers(),
    {&autoNavBeybladeCommand, &sentryTurretCVCommand},
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::UP));

HoldRepeatCommandMapping shootRightSwitchMidAgitator(
    drivers(),
    {&girlbossRotateAndUnjamAgitatorWithCVAndHeatLimiting, &malewifeRotateAndUnjamAgitatorWithCVAndHeatLimiting},
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::MID),
    true);

HoldRepeatCommandMapping shootRightSwitchUpAgitator(
    drivers(),
    {&girlbossRotateAndUnjamAgitatorWithCVAndHeatLimiting, &malewifeRotateAndUnjamAgitatorWithCVAndHeatLimiting},
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::UP),
    true);

bool isInitialized = false;

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    sentryDrive.initialize();
    turretMajor.initialize();
    turretMinorGirlboss.initialize();
    turretMinorMalewife.initialize();
    odometrySubsystem.initialize();
    sentryTransforms.initialize();
    // turret
    frictionWheelsGirlboss.initialize();
    girlbossAgitator.initialize();
    frictionWheelsMalewife.initialize();
    malewifeAgitator.initialize();

    arucoResetSubsystem.initialize();
    sentryResponseSubsystem.initialize();

    isInitialized = true;
}

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* register subsystems here -------------------------------------------------*/
void registerSentrySubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&sentryDrive);
    drivers->commandScheduler.registerSubsystem(&turretMinorGirlboss);
    drivers->commandScheduler.registerSubsystem(&turretMinorMalewife);
    drivers->commandScheduler.registerSubsystem(&turretMajor);
    drivers->commandScheduler.registerSubsystem(&odometrySubsystem);
    drivers->commandScheduler.registerSubsystem(&arucoResetSubsystem);
    drivers->commandScheduler.registerSubsystem(&sentryTransforms);
    drivers->commandScheduler.registerSubsystem(&frictionWheelsGirlboss);
    drivers->commandScheduler.registerSubsystem(&frictionWheelsMalewife);

    drivers->commandScheduler.registerSubsystem(&girlbossAgitator);
    drivers->commandScheduler.registerSubsystem(&malewifeAgitator);
    drivers->visionCoprocessor.attachSentryTransformer(&sentryTransforms);

    drivers->commandScheduler.registerSubsystem(&sentryResponseSubsystem);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSentryCommands(Drivers *)
{
    sentryDrive.setDefaultCommand(&chassisDriveCommand);
    turretMajor.setDefaultCommand(&turretMajorControlCommand);
    turretMinorGirlboss.setDefaultCommand(&turretMinorGirlbossControlCommand);  // @todo https://gitlab.com/aruw/controls/taproot/-/issues/215
    turretMinorMalewife.setDefaultCommand(&turretMinorMalewifeControlCommand);

    frictionWheelsGirlboss.setDefaultCommand(&stopGirlBossFrictionWheelSpinCommand);
    frictionWheelsMalewife.setDefaultCommand(&stopMaleWifeFrictionWheelSpinCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSentryCommands(Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&imuCalibrateCommand);
    // @todo does not belong here
    sentryStrategyRequestHandler.attachNoStrategyHandler(&sendNoMotionStrategy);
    sentryStrategyRequestHandler.attachGoToFriendlyBaseHandler(&sendGoToFriendlyBaseStrategy);
    sentryStrategyRequestHandler.attachGoToEnemyBaseHandler(&sendGoToEnemyBaseStrategy);
    sentryStrategyRequestHandler.attachGoToSupplierZoneHandler(&sendGoToFriendlySupplierZoneStrategy);
    sentryStrategyRequestHandler.attachGoToEnemySupplierZoneHandler(&sendGoToEnemySupplierZoneStrategy);
    sentryStrategyRequestHandler.attachGoToCenterPointHandler(&sendGoToCenterPointStrategy);

    // new commands to receive
    sentryStrategyRequestHandler.attachHoldFireHandler(&holdFireCallback);
    sentryStrategyRequestHandler.attachToggleMovementHandler(&toggleMovementCallback);
    sentryStrategyRequestHandler.attachToggleBeybladeHandler(&toggleBeybladeCallback);
    sentryStrategyRequestHandler.attachHoldFireHandler(&holdFireCallback);

    // new commands to receive


    drivers->refSerial.attachRobotToRobotMessageHandler(
        aruwsrc::communication::serial::SENTRY_STRATEGY_REQUEST_ID,
        &sentryStrategyRequestHandler);
}

/* register io mappings here ------------------------------------------------*/
void registerSentryIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(&manualRightSwitchUp);
    drivers->commandMapper.addMap(&autoRightSwitchMid);  // @todo: https://gitlab.com/aruw/controls/taproot/-/issues/219
    drivers->commandMapper.addMap(&autoRightSwitchUp);
    drivers->commandMapper.addMap(&autoRightSwitchDown);
    drivers->commandMapper.addMap(&shootRightSwitchUpAgitator);
    drivers->commandMapper.addMap(&shootRightSwitchUp);
    drivers->commandMapper.addMap(&shootRightSwitchMidAgitator);
    drivers->commandMapper.addMap(&shootRightSwitchMid);
    drivers->commandMapper.addMap(&shootRightSwitchDownAgitator);
    drivers->commandMapper.addMap(&shoot);
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
// imu::ImuCalibrateCommand *getImuCalibrateCommand() { return &sentry_control::imuCalibrateCommand;
// }
#endif

#endif
