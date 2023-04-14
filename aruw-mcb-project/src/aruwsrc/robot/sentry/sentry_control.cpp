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

#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"
#include "aruwsrc/communication/low_battery_buzzer_command.hpp"
#include "aruwsrc/communication/mcb-lite/motor/virtual_dji_motor.hpp"
#include "aruwsrc/communication/mcb-lite/virtual_mcb_handler.hpp"
#include "aruwsrc/communication/serial/sentry_request_handler.hpp"
#include "aruwsrc/communication/serial/sentry_request_message_types.hpp"
#include "aruwsrc/communication/serial/sentry_response_subsystem.hpp"
#include "aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "aruwsrc/control/agitator/constants/agitator_constants.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/auto-aim/auto_aim_fire_rate_reselection_manager.hpp"
#include "aruwsrc/control/buzzer/buzzer_subsystem.hpp"
#include "aruwsrc/control/chassis/beyblade_command.hpp"
#include "aruwsrc/control/chassis/chassis_autorotate_command.hpp"
#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "aruwsrc/control/chassis/chassis_imu_drive_command.hpp"
#include "aruwsrc/control/chassis/mecanum_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/sentry/sentry_auto_drive_comprised_command.hpp"
#include "aruwsrc/control/chassis/sentry/sentry_drive_manual_command.hpp"
#include "aruwsrc/control/chassis/sentry/sentry_drive_subsystem.hpp"
#include "aruwsrc/control/chassis/swerve_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/wiggle_drive_command.hpp"
#include "aruwsrc/control/governor/cv_has_target_governor.hpp"
#include "aruwsrc/control/governor/cv_on_target_governor.hpp"
#include "aruwsrc/control/governor/cv_online_governor.hpp"
#include "aruwsrc/control/governor/fire_rate_limit_governor.hpp"
#include "aruwsrc/control/governor/friction_wheels_on_governor.hpp"
#include "aruwsrc/control/governor/heat_limit_governor.hpp"
#include "aruwsrc/control/governor/pause_command_governor.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "aruwsrc/control/launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/cv/sentry_turret_cv_command.hpp"
#include "aruwsrc/control/turret/user/turret_quick_turn_command.hpp"
#include "aruwsrc/control/turret/user/turret_user_control_command.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/sentry/sentry_otto_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_major_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_minor_subsystem.hpp"

using namespace tap::control::governor;
using namespace tap::control::setpoint;
using namespace aruwsrc::agitator;
using namespace aruwsrc::control::sentry::drive;
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
using namespace tap::communication::serial;
using namespace aruwsrc::sentry;

namespace sentry_control
{

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

aruwsrc::communication::serial::SentryRequestHandler sentryRequestHandler(drivers());

// forward declare before sentry turret to be used in turret CV command
extern SentryOttoKFOdometry2DSubsystem odometrySubsystem;

class SentryMinorTurret
{
public:
    struct Config
    {
        aruwsrc::agitator::VelocityAgitatorSubsystemConfig agitatorConfig;
        TurretMotorConfig pitchMotorConfig;
        TurretMotorConfig yawMotorConfig;
        tap::can::CanBus turretCanBus;
        bool pitchMotorInverted;
        uint8_t turretID;
        RefSerialData::Rx::MechanismID turretBarrelMechanismId;
        tap::algorithms::SmoothPidConfig pitchPidConfig;
        tap::algorithms::SmoothPidConfig yawPidConfig;
        tap::algorithms::SmoothPidConfig yawPosPidConfig;
        tap::algorithms::SmoothPidConfig yawVelPidConfig;
        aruwsrc::can::TurretMCBCanComm &turretMCBCanComm;
    };

    SentryMinorTurret(Drivers &drivers, const Config &config)
        : agitator(&drivers, constants::AGITATOR_PID_CONFIG, config.agitatorConfig),
          frictionWheels(
              &drivers,
              LEFT_MOTOR_ID,
              RIGHT_MOTOR_ID,
              config.turretCanBus,
              &config.turretMCBCanComm,
              config.turretBarrelMechanismId),
          pitchMotor(
              &drivers,
              aruwsrc::control::turret::PITCH_MOTOR_ID,
              config.turretCanBus,
              config.pitchMotorInverted,
              "Pitch Turret"),
          yawMotor(
              &drivers,
              aruwsrc::control::turret::YAW_MOTOR_ID,
              config.turretCanBus,
              true,
              "Yaw Turret"),
          turretSubsystem(
              &drivers,
              &pitchMotor,
              &yawMotor,
              config.pitchMotorConfig,
              config.yawMotorConfig,
              &config.turretMCBCanComm,
              config.turretID),
          ballisticsSolver(
              drivers.visionCoprocessor,
              odometrySubsystem,
              turretSubsystem,
              frictionWheels,
              29.5f,  // defaultLaunchSpeed
              config.turretID),
          autoAimLaunchTimer(
              aruwsrc::control::launcher::AGITATOR_TYPICAL_DELAY_MICROSECONDS,
              &drivers.visionCoprocessor,
              &ballisticsSolver),
          spinFrictionWheels(
              &drivers,
              &frictionWheels,
              30.0f,
              true,
              config.turretBarrelMechanismId),
          stopFrictionWheels(&drivers, &frictionWheels, 0.0f, true, config.turretBarrelMechanismId),
          chassisFramePitchTurretController(turretSubsystem.pitchMotor, config.pitchPidConfig),
          chassisFrameYawTurretController(turretSubsystem.yawMotor, config.yawPidConfig),
          worldFrameYawTurretImuPosPid(config.yawPosPidConfig),
          worldFrameYawTurretImuVelPid(config.yawVelPidConfig),
          worldFrameYawTurretImuController(
              config.turretMCBCanComm,
              turretSubsystem.yawMotor,
              worldFrameYawTurretImuPosPid,
              worldFrameYawTurretImuVelPid),
          turretManual(
              &drivers,
              drivers.controlOperatorInterface,
              &turretSubsystem,
              &worldFrameYawTurretImuController,
              &chassisFramePitchTurretController,
              USER_YAW_INPUT_SCALAR,
              USER_PITCH_INPUT_SCALAR,
              config.turretID),
          turretCVCommand(
              &drivers.visionCoprocessor,
              &turretSubsystem,
              &worldFrameYawTurretImuController,
              &chassisFramePitchTurretController,
              &ballisticsSolver,
              config.turretID),
          turretUturnCommand(&turretSubsystem, M_PI),
          rotateAgitator(agitator, constants::AGITATOR_ROTATE_CONFIG),
          unjamAgitator(agitator, constants::AGITATOR_UNJAM_CONFIG),
          rotateAndUnjamAgitator(drivers, agitator, rotateAgitator, unjamAgitator),
          frictionWheelsOnGovernor(frictionWheels),
          heatLimitGovernor(drivers, config.turretBarrelMechanismId, constants::HEAT_LIMIT_BUFFER),
          cvOnTargetGovernor(
              &drivers,
              drivers.visionCoprocessor,
              turretCVCommand,
              autoAimLaunchTimer,
              CvOnTargetGovernorMode::ON_TARGET_AND_GATED),
          cvOnlineGovernor(drivers, drivers.visionCoprocessor, turretCVCommand),
          autoAimFireRateManager(
              drivers,
              drivers.visionCoprocessor,
              drivers.commandScheduler,
              turretCVCommand,
              config.turretID),
          fireRateLimitGovernor(autoAimFireRateManager),
          pauseCommandGovernor(
              aruwsrc::control::agitator::constants::AGITATOR_PAUSE_PROJECTILE_LAUNCHING_TIME),
          rotateAndUnjamAgitatorWithHeatAndCvLimitingWhenCvOnline(
              {&agitator},
              rotateAndUnjamAgitator,
              {&heatLimitGovernor,
               &frictionWheelsOnGovernor,
               &cvOnTargetGovernor,
               &cvOnlineGovernor,
               &fireRateLimitGovernor,
               &pauseCommandGovernor}),
          rotateAndUnjamAgitatorWithHeatAndCvLimiting(
              {&agitator},
              rotateAndUnjamAgitator,
              {&heatLimitGovernor,
               &frictionWheelsOnGovernor,
               &cvOnTargetGovernor,
               &fireRateLimitGovernor})
    {
    }

    // subsystems
    VelocityAgitatorSubsystem agitator;
    RefereeFeedbackFrictionWheelSubsystem<LAUNCH_SPEED_AVERAGING_DEQUE_SIZE> frictionWheels;
    DjiMotor pitchMotor;
    DjiMotor yawMotor;
    SentryTurretMinorSubsystem turretSubsystem;

    OttoBallisticsSolver ballisticsSolver;
    AutoAimLaunchTimer autoAimLaunchTimer;

    // friction wheel commands
    FrictionWheelSpinRefLimitedCommand spinFrictionWheels;
    FrictionWheelSpinRefLimitedCommand stopFrictionWheels;

    // turret controllers
    algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController;
    algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController;

    tap::algorithms::SmoothPid worldFrameYawTurretImuPosPid;
    tap::algorithms::SmoothPid worldFrameYawTurretImuVelPid;
    algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController;

    // turret commands
    // limits fire rate
    user::TurretUserControlCommand turretManual;

    cv::SentryTurretCVCommand turretCVCommand;

    user::TurretQuickTurnCommand turretUturnCommand;

    // base agitator commands
    MoveIntegralCommand rotateAgitator;
    UnjamIntegralCommand unjamAgitator;
    MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator;

    // agitator related governors
    FrictionWheelsOnGovernor frictionWheelsOnGovernor;
    HeatLimitGovernor heatLimitGovernor;
    CvOnTargetGovernor cvOnTargetGovernor;
    CvOnlineGovernor cvOnlineGovernor;
    AutoAimFireRateReselectionManager autoAimFireRateManager;
    FireRateLimitGovernor fireRateLimitGovernor;
    PauseCommandGovernor pauseCommandGovernor;

    // agitator governor limited commands
    GovernorLimitedCommand<6> rotateAndUnjamAgitatorWithHeatAndCvLimitingWhenCvOnline;
    GovernorLimitedCommand<4> rotateAndUnjamAgitatorWithHeatAndCvLimiting;
};

aruwsrc::virtualMCB::VirtualDjiMotor motor1(
    drivers(),
    MOTOR1,
    CAN_BUS1,
    &(drivers()->mcbLite),
    false,
    "swerve virtual motor 1");

aruwsrc::virtualMCB::VirtualDjiMotor motor2(
    drivers(),
    MOTOR1,
    CAN_BUS1,
    &(drivers()->mcbLite),
    false,
    "swerve virtual motor 2");

aruwsrc::virtualMCB::VirtualDjiMotor motor3(
    drivers(),
    MOTOR1,
    CAN_BUS1,
    &(drivers()->mcbLite),
    false,
    "swerve virtual motor 3");

aruwsrc::virtualMCB::VirtualDjiMotor motor4(
    drivers(),
    MOTOR1,
    CAN_BUS1,
    &(drivers()->mcbLite),
    false,
    "swerve virtual motor 4");

aruwsrc::virtualMCB::VirtualDjiMotor motor5(
    drivers(),
    MOTOR1,
    CAN_BUS1,
    &(drivers()->mcbLite),
    false,
    "swerve virtual motor 5");

aruwsrc::virtualMCB::VirtualDjiMotor motor6(
    drivers(),
    MOTOR1,
    CAN_BUS1,
    &(drivers()->mcbLite),
    false,
    "swerve virtual motor 6");

aruwsrc::virtualMCB::VirtualDjiMotor motor7(
    drivers(),
    MOTOR1,
    CAN_BUS1,
    &(drivers()->mcbLite),
    false,
    "swerve virtual motor 7");

aruwsrc::virtualMCB::VirtualDjiMotor motor8(
    drivers(),
    MOTOR1,
    CAN_BUS1,
    &(drivers()->mcbLite),
    false,
    "swerve virtual motor 8");

tap::motor::DjiMotor turretMajorYawMotor(
    drivers(),
    YAW_MOTOR_ID,
    CAN_BUS_MOTORS,
    true,
    "Major Yaw Turret");

/* define subsystems --------------------------------------------------------*/

aruwsrc::chassis::SwerveChassisSubsystem sentryDrive(drivers());

SentryTurretMajorSubsystem turretMajor(drivers(), &turretMajorYawMotor, majorYawConfig);

SentryMinorTurret turretZero(
    *drivers(),
    {
        .agitatorConfig = aruwsrc::control::agitator::constants::turret0::AGITATOR_CONFIG,
        .pitchMotorConfig = aruwsrc::control::turret::turret0::PITCH_MOTOR_CONFIG,
        .yawMotorConfig = aruwsrc::control::turret::turret0::YAW_MOTOR_CONFIG,
        .turretCanBus = aruwsrc::control::turret::turret0::CAN_BUS_MOTORS,
        .pitchMotorInverted = false,
        .turretID = 0,
        .turretBarrelMechanismId = RefSerialData::Rx::MechanismID::TURRET_17MM_2,
        .pitchPidConfig = aruwsrc::control::turret::chassis_rel::turret0::PITCH_PID_CONFIG,
        .yawPidConfig = aruwsrc::control::turret::chassis_rel::turret0::YAW_PID_CONFIG,
        .yawPosPidConfig = world_rel_turret_imu::turret0::YAW_POS_PID_CONFIG,
        .yawVelPidConfig = world_rel_turret_imu::turret0::YAW_VEL_PID_CONFIG,
        .turretMCBCanComm = drivers()->turretMCBCanCommBus2,
    });

SentryMinorTurret turretOne(
    *drivers(),
    {
        .agitatorConfig = aruwsrc::control::agitator::constants::turret1::AGITATOR_CONFIG,
        .pitchMotorConfig = aruwsrc::control::turret::turret1::PITCH_MOTOR_CONFIG,
        .yawMotorConfig = aruwsrc::control::turret::turret1::YAW_MOTOR_CONFIG,
        .turretCanBus = aruwsrc::control::turret::turret1::CAN_BUS_MOTORS,
        .pitchMotorInverted = true,
        .turretID = 1,
        .turretBarrelMechanismId = RefSerialData::Rx::MechanismID::TURRET_17MM_1,
        .pitchPidConfig = aruwsrc::control::turret::chassis_rel::turret1::PITCH_PID_CONFIG,
        .yawPidConfig = aruwsrc::control::turret::chassis_rel::turret1::YAW_PID_CONFIG,
        .yawPosPidConfig = world_rel_turret_imu::turret1::YAW_POS_PID_CONFIG,
        .yawVelPidConfig = world_rel_turret_imu::turret1::YAW_VEL_PID_CONFIG,
        .turretMCBCanComm = drivers()->turretMCBCanCommBus1,
    });

/* define commands ----------------------------------------------------------*/
imu::ImuCalibrateCommand imuCalibrateCommand(
    drivers(),
    {
        {
            &drivers()->turretMCBCanCommBus2,
            &turretZero.turretSubsystem,
            &turretZero.chassisFrameYawTurretController,
            &turretZero.chassisFramePitchTurretController,
            false,
        },
        {
            &drivers()->turretMCBCanCommBus1,
            &turretOne.turretSubsystem,
            &turretOne.chassisFrameYawTurretController,
            &turretOne.chassisFramePitchTurretController,
            false,
        },
    },
    &sentryDrive);

aruwsrc::chassis::ChassisDriveCommand chassisDriveCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &sentryDrive);

aruwsrc::chassis::ChassisAutorotateCommand chassisAutorotateCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &sentryDrive,
    &turretMajor.yawMotor,
    aruwsrc::chassis::ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180);

aruwsrc::chassis::WiggleDriveCommand wiggleCommand(
    drivers(),
    &sentryDrive,
    &turretMajor.yawMotor,
    (drivers()->controlOperatorInterface));
aruwsrc::chassis::BeybladeCommand beybladeCommand(
    drivers(),
    &sentryDrive,
    &turretMajor.yawMotor,
    (drivers()->controlOperatorInterface));

void selectNewRobotMessageHandler() { drivers()->visionCoprocessor.sendSelectNewTargetMessage(); }

void targetNewQuadrantMessageHandler()
{
    turretZero.turretCVCommand.changeScanningQuadrant();
    turretOne.turretCVCommand.changeScanningQuadrant();
}

// void toggleDriveMovementMessageHandler() { sentryAutoDrive.toggleDriveMovement(); }

void pauseProjectileLaunchMessageHandler()
{
    turretZero.pauseCommandGovernor.initiatePause();
    turretOne.pauseCommandGovernor.initiatePause();
}

/* define command mappings --------------------------------------------------*/

HoldCommandMapping rightSwitchDown(
    drivers(),
    {&turretZero.stopFrictionWheels, &turretOne.stopFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&turretZero.rotateAndUnjamAgitatorWithHeatAndCvLimiting,
     &turretOne.rotateAndUnjamAgitatorWithHeatAndCvLimiting},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    true);
HoldCommandMapping leftSwitchDown(
    drivers(),
    {&sentryDriveManual1, &turretZero.turretManual, &turretOne.turretManual},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
HoldCommandMapping leftSwitchMid(
    drivers(),
    {&sentryDriveManual2},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    sentryDrive.initialize();
    turretZero.agitator.initialize();
    turretZero.frictionWheels.initialize();
    turretZero.turretSubsystem.initialize();
    turretOne.agitator.initialize();
    turretOne.frictionWheels.initialize();
    turretOne.turretSubsystem.initialize();
    odometrySubsystem.initialize();
}

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* register subsystems here -------------------------------------------------*/
void registerSentrySubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&sentryDrive);
    drivers->commandScheduler.registerSubsystem(&turretZero.agitator);
    drivers->commandScheduler.registerSubsystem(&turretZero.frictionWheels);
    drivers->commandScheduler.registerSubsystem(&turretZero.turretSubsystem);
    drivers->commandScheduler.registerSubsystem(&turretOne.agitator);
    drivers->commandScheduler.registerSubsystem(&turretOne.frictionWheels);
    drivers->commandScheduler.registerSubsystem(&turretOne.turretSubsystem);
    drivers->commandScheduler.registerSubsystem(&odometrySubsystem);
    drivers->visionCoprocessor.attachOdometryInterface(&odometrySubsystem);
    drivers->visionCoprocessor.attachTurretOrientationInterface(&turretZero.turretSubsystem, 0);
    drivers->visionCoprocessor.attachTurretOrientationInterface(&turretOne.turretSubsystem, 1);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSentryCommands(Drivers *)
{
    sentryDrive.setDefaultCommand(&chassisAutorotateCommand);
    turretZero.frictionWheels.setDefaultCommand(&turretZero.spinFrictionWheels);
    turretOne.frictionWheels.setDefaultCommand(&turretOne.spinFrictionWheels);
    turretZero.turretSubsystem.setDefaultCommand(&turretZero.turretCVCommand);
    turretOne.turretSubsystem.setDefaultCommand(&turretOne.turretCVCommand);
    turretZero.agitator.setDefaultCommand(
        &turretZero.rotateAndUnjamAgitatorWithHeatAndCvLimitingWhenCvOnline);
    turretOne.agitator.setDefaultCommand(
        &turretOne.rotateAndUnjamAgitatorWithHeatAndCvLimitingWhenCvOnline);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSentryCommands(Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&imuCalibrateCommand);

    sentryRequestHandler.attachPauseProjectileLaunchingMessageHandler(
        pauseProjectileLaunchMessageHandler);
    sentryRequestHandler.attachSelectNewRobotMessageHandler(selectNewRobotMessageHandler);
    sentryRequestHandler.attachTargetNewQuadrantMessageHandler(targetNewQuadrantMessageHandler);
    drivers->refSerial.attachRobotToRobotMessageHandler(
        aruwsrc::communication::serial::SENTRY_REQUEST_ROBOT_ID,
        &sentryRequestHandler);
}

/* register io mappings here ------------------------------------------------*/
void registerSentryIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchMid);
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
