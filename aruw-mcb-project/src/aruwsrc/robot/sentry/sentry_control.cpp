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

// #include "aruwsrc/algorithms/otto_ballistics_solver.hpp"
// #include "aruwsrc/communication/low_battery_buzzer_command.hpp"
#include "aruwsrc/communication/mcb-lite/motor/virtual_dji_motor.hpp"
#include "aruwsrc/communication/mcb-lite/virtual_mcb_handler.hpp"
#include "aruwsrc/communication/serial/sentry_request_handler.hpp"
#include "aruwsrc/communication/serial/sentry_request_message_types.hpp"
// #include "aruwsrc/communication/serial/sentry_response_subsystem.hpp"
// #include "aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "aruwsrc/control/agitator/constants/agitator_constants.hpp"
// #include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
// #include "aruwsrc/control/auto-aim/auto_aim_fire_rate_reselection_manager.hpp"
// #include "aruwsrc/control/buzzer/buzzer_subsystem.hpp"
// #include "aruwsrc/control/chassis/beyblade_command.hpp"
// #include "aruwsrc/control/chassis/chassis_autorotate_command.hpp"
// #include "aruwsrc/control/chassis/chassis_drive_command.hpp"
// #include "aruwsrc/control/chassis/chassis_imu_drive_command.hpp"
// #include "aruwsrc/control/chassis/mecanum_chassis_subsystem.hpp"
// #include "aruwsrc/control/chassis/sentry/sentry_auto_drive_comprised_command.hpp"
// #include "aruwsrc/control/chassis/sentry/sentry_drive_manual_command.hpp"
// #include "aruwsrc/control/chassis/sentry/sentry_drive_subsystem.hpp"
#include "aruwsrc/control/chassis/swerve_chassis_subsystem.hpp"
// #include "aruwsrc/control/chassis/swerve_module_config.hpp"
// #include "aruwsrc/control/chassis/wiggle_drive_command.hpp"
// #include "aruwsrc/control/governor/cv_has_target_governor.hpp"
// #include "aruwsrc/control/governor/cv_on_target_governor.hpp"
// #include "aruwsrc/control/governor/cv_online_governor.hpp"
// #include "aruwsrc/control/governor/fire_rate_limit_governor.hpp"
// #include "aruwsrc/control/governor/friction_wheels_on_governor.hpp"
// #include "aruwsrc/control/governor/heat_limit_governor.hpp"
// #include "aruwsrc/control/governor/pause_command_governor.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
// #include "aruwsrc/control/launcher/friction_wheel_spin_ref_limited_command.hpp"
// #include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
// #include "aruwsrc/control/safe_disconnect.hpp"
// #include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
// #include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
// #include "aruwsrc/control/turret/constants/turret_constants.hpp"
// #include "aruwsrc/control/turret/cv/sentry_turret_cv_command.hpp"
// #include "aruwsrc/control/turret/user/turret_quick_turn_command.hpp"
// #include "aruwsrc/control/turret/user/turret_user_control_command.hpp"
#include "aruwsrc/drivers_singleton.hpp"
// #include "aruwsrc/robot/sentry/sentry_otto_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_major_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_minor_subsystem.hpp"

#include "aruwsrc/robot/sentry/sentry_turret_minor_governor.hpp"
#include "aruwsrc/control/chassis/swerve_module.hpp"
#include "aruwsrc/robot/sentry/sentry_beehive_chassis_constants.hpp"
#include "aruwsrc/control/chassis/new_sentry/sentry_manual_drive_command.hpp"
#include "aruwsrc/control/turret/sentry/turret_major_sentry_control_command.hpp"
#include "aruwsrc/control/turret/sentry/turret_minor_sentry_control_command.hpp"
#include "aruwsrc/control/turret/sentry/turret_minor_sentry_control_command.hpp"

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
// using namespace aruwsrc::algorithms::odometry;
using namespace aruwsrc::algorithms;
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

aruwsrc::communication::serial::SentryRequestHandler sentryRequestHandler(drivers());

/* define swerve motors --------------------------------------------------------*/

VirtualDjiMotor leftFrontDriveMotor(
    drivers(),
    MOTOR1,
    aruwsrc::sentry::chassis::CAN_BUS_MOTORS,
    &(drivers()->mcbLite),
    false,
    "Left Front Swerve Drive Motor");

VirtualDjiMotor leftFrontAzimuthMotor(
    drivers(),
    MOTOR5,
    aruwsrc::sentry::chassis::CAN_BUS_MOTORS,
    &(drivers()->mcbLite),
    false,
    "Left Front Swerve Azimuth Motor");

VirtualDjiMotor rightFrontDriveMotor(
    drivers(),
    MOTOR4,
    aruwsrc::sentry::chassis::CAN_BUS_MOTORS,
    &(drivers()->mcbLite),
    false,
    "Right Front Swerve Drive Motor");

VirtualDjiMotor rightFrontAzimuthMotor(
    drivers(),
    MOTOR8,
    aruwsrc::sentry::chassis::CAN_BUS_MOTORS,
    &(drivers()->mcbLite),
    false,
    "Right Front Swerve Azimuth Motor");

VirtualDjiMotor leftBackDriveMotor(
    drivers(),
    MOTOR2,
    aruwsrc::sentry::chassis::CAN_BUS_MOTORS,
    &(drivers()->mcbLite),
    false,
    "Left Back Swerve Drive Motor");

VirtualDjiMotor leftBackAzimuthMotor(
    drivers(),
    MOTOR6,
    aruwsrc::sentry::chassis::CAN_BUS_MOTORS,
    &(drivers()->mcbLite),
    false,
    "Left Back Swerve Azimuth Motor");

VirtualDjiMotor rightBackDriveMotor(
    drivers(),
    MOTOR3,
    aruwsrc::sentry::chassis::CAN_BUS_MOTORS,
    &(drivers()->mcbLite),
    false,
    "Right Back Swerve Drive Motor");

VirtualDjiMotor rightBackAzimuthMotor(
    drivers(),
    MOTOR7,
    aruwsrc::sentry::chassis::CAN_BUS_MOTORS,
    &(drivers()->mcbLite),
    false,
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
    false,
    false,
    "Major Yaw Turret 1",
    "Major Yaw Turret 2"
);

tap::motor::DjiMotor turretMinor0YawMotor(
    drivers(),
    MOTOR6,
    turretMinor0::CAN_BUS_MOTORS,
    false,
    "Minor 0 Yaw Turret"
);

tap::motor::DjiMotor turretMinor0PitchMotor(
    drivers(),
    MOTOR5,
    turretMinor0::CAN_BUS_MOTORS,
    false,
    "Minor 0 Pitch Turret"
);

tap::motor::DjiMotor turretMinor1YawMotor(
    drivers(),
    MOTOR6,
    turretMinor1::CAN_BUS_MOTORS,
    false,
    "Minor 1 Yaw Turret"
);

tap::motor::DjiMotor turretMinor1PitchMotor(
    drivers(),
    MOTOR5,
    turretMinor1::CAN_BUS_MOTORS,
    true,
    "Minor 1 Pitch Turret"
);

/* define subsystems --------------------------------------------------------*/

aruwsrc::chassis::SwerveChassisSubsystem sentryDrive(
    drivers(),
    &leftFrontSwerveModule,
    &rightFrontSwerveModule,
    &leftBackSwerveModule,
    &rightBackSwerveModule,
    aruwsrc::sentry::chassis::SWERVE_FORWARD_MATRIX);

SentryTurretMajorSubsystem turretMajor(drivers(), &turretMajorYawMotor, YAW_MOTOR_CONFIG);

// Because there is no governor for the turret major, we need to instantiate
// a yaw controller for the turret major ourselves
algorithms::ChassisFrameYawTurretController turretMajorYawController = algorithms::ChassisFrameYawTurretController(
    turretMajor.yawMotor,
    aruwsrc::control::turret::chassis_rel::turretMajor::YAW_PID_CONFIG
);

SentryMinorTurretGovernor turretZero(
    *drivers(),
    {
        .agitatorConfig = aruwsrc::control::agitator::constants::turretMinor0::AGITATOR_CONFIG,
        .pitchMotor = &turretMinor0PitchMotor,
        .yawMotor = &turretMinor0YawMotor,
        .pitchMotorConfig = aruwsrc::control::turret::turretMinor0::PITCH_MOTOR_CONFIG,
        .yawMotorConfig = aruwsrc::control::turret::turretMinor0::YAW_MOTOR_CONFIG,
        .turretCanBus = aruwsrc::control::turret::turretMinor0::CAN_BUS_MOTORS,
        .turretID = 0,
        .turretBarrelMechanismId = RefSerialData::Rx::MechanismID::TURRET_17MM_2,
        .pitchPidConfig = aruwsrc::control::turret::chassis_rel::turretMinor0::PITCH_PID_CONFIG,
        .yawPidConfig = aruwsrc::control::turret::chassis_rel::turretMinor0::YAW_PID_CONFIG,
        .yawPosPidConfig = world_rel_turret_imu::turretMinor0::YAW_POS_PID_CONFIG,
        .yawVelPidConfig = world_rel_turret_imu::turretMinor0::YAW_VEL_PID_CONFIG,
        .turretMCBCanComm = drivers()->turretMCBCanCommBus1,
    });

SentryMinorTurretGovernor turretOne(
    *drivers(),
    {
        .agitatorConfig = aruwsrc::control::agitator::constants::turretMinor1::AGITATOR_CONFIG,
        .pitchMotor = &turretMinor1PitchMotor,
        .yawMotor = &turretMinor1YawMotor,
        .pitchMotorConfig = aruwsrc::control::turret::turretMinor1::PITCH_MOTOR_CONFIG,
        .yawMotorConfig = aruwsrc::control::turret::turretMinor1::YAW_MOTOR_CONFIG,
        .turretCanBus = aruwsrc::control::turret::turretMinor1::CAN_BUS_MOTORS,
        .turretID = 1,
        .turretBarrelMechanismId = RefSerialData::Rx::MechanismID::TURRET_17MM_1,
        .pitchPidConfig = aruwsrc::control::turret::chassis_rel::turretMinor1::PITCH_PID_CONFIG,
        .yawPidConfig = aruwsrc::control::turret::chassis_rel::turretMinor1::YAW_PID_CONFIG,
        .yawPosPidConfig = world_rel_turret_imu::turretMinor1::YAW_POS_PID_CONFIG,
        .yawVelPidConfig = world_rel_turret_imu::turretMinor1::YAW_VEL_PID_CONFIG,
        .turretMCBCanComm = drivers()->turretMCBCanCommBus2,
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

aruwsrc::control::sentry::SentryManualDriveCommand chassisDriveCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &sentryDrive);

aruwsrc::control::turret::sentry::TurretMajorSentryControlCommand turretMajorControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turretMajor,
    &turretMajorYawController,
    aruwsrc::control::turret::MAJOR_USER_YAW_INPUT_SCALAR
);

aruwsrc::control::turret::sentry::TurretMinorSentryControlCommand turretMinor0ControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turretZero.turretSubsystem,
    &turretZero.chassisFrameYawTurretController,
    &turretZero.chassisFramePitchTurretController,
    MINOR_USER_YAW_INPUT_SCALAR,
    MINOR_USER_PITCH_INPUT_SCALAR,
    0);

aruwsrc::control::turret::sentry::TurretMinorSentryControlCommand turretMinor1ControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turretOne.turretSubsystem,
    &turretOne.chassisFrameYawTurretController,
    &turretOne.chassisFramePitchTurretController,
    MINOR_USER_YAW_INPUT_SCALAR,
    MINOR_USER_PITCH_INPUT_SCALAR,
    1);

// aruwsrc::control::turret::sentry::TurretMinorSentryWorldRelativeCommand turretMinor0ControlCommand(
//     drivers(),
//     &turretZero.turretSubsystem,
//     &turretZero.chassisFrameYawTurretController,
//     &turretZero.chassisFramePitchTurretController,
// );


// aruwsrc::control::turret::sentry::TurretMinorSentryWorldRelativeCommand turretMinor1ControlCommand(
//     drivers(),
//     &turretOne.turretSubsystem,
//     &turretOne.chassisFrameYawTurretController,
//     &turretOne.chassisFramePitchTurretController,
//     USER_YAW_INPUT_SCALAR,
// );
// aruwsrc::chassis::ChassisAutorotateCommand chassisAutorotateCommand(
//     drivers(),
//     &drivers()->controlOperatorInterface,
//     &sentryDrive,
//     &turretMajor.yawMotor,
//     aruwsrc::chassis::ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180);

// aruwsrc::chassis::WiggleDriveCommand wiggleCommand(
//     drivers(),
//     &sentryDrive,
//     &turretMajor.yawMotor,
//     (drivers()->controlOperatorInterface));
// aruwsrc::chassis::BeybladeCommand beybladeCommand(
//     drivers(),
//     &sentryDrive,
//     &turretMajor.yawMotor,
//     (drivers()->controlOperatorInterface));




// void selectNewRobotMessageHandler() { drivers()->visionCoprocessor.sendSelectNewTargetMessage(); }

void targetNewQuadrantMessageHandler()
{
    // turretZero.turretCVCommand.changeScanningQuadrant();
    // turretOne.turretCVCommand.changeScanningQuadrant();
}

// void toggleDriveMovementMessageHandler() { sentryAutoDrive.toggleDriveMovement(); }

void pauseProjectileLaunchMessageHandler()
{
    // turretZero.pauseCommandGovernor.initiatePause();
    // turretOne.pauseCommandGovernor.initiatePause();
}

/* define command mappings --------------------------------------------------*/

// We're currently going to ignore right switch inputs.  TODO: Change this back.
// HoldCommandMapping rightSwitchDown(
//     drivers(),
//     {&turretZero.stopFrictionWheels, &turretOne.stopFrictionWheels},
//     RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
// HoldRepeatCommandMapping rightSwitchUp(
//     drivers(),
//     {&turretZero.rotateAndUnjamAgitatorWithHeatAndCvLimiting,
//      &turretOne.rotateAndUnjamAgitatorWithHeatAndCvLimiting},
//     RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
//     true);




HoldCommandMapping leftSwitchDown(
    drivers(),
    {&chassisDriveCommand, &turretMajorControlCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
HoldCommandMapping leftSwitchMid(
    drivers(),
    {&turretMinor0ControlCommand, &turretMinor1ControlCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));





/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    sentryDrive.initialize();
    // turretZero.agitator.initialize();
    // turretZero.frictionWheels.initialize();
    turretZero.turretSubsystem.initialize();
    // turretOne.agitator.initialize();
    // turretOne.frictionWheels.initialize();
    turretOne.turretSubsystem.initialize();

    turretMajor.initialize();
    // odometrySubsystem.initialize();
    // turret
 
    // leftFrontDriveMotor.setDesiredOutput(500);
    // leftFrontAzimuthMotor.setDesiredOutput(500);
    // rightFrontDriveMotor.setDesiredOutput(500);
    // rightFrontAzimuthMotor.setDesiredOutput(500);
    // leftBackDriveMotor.setDesiredOutput(500);
    // leftBackAzimuthMotor.setDesiredOutput(500);
    // rightBackDriveMotor.setDesiredOutput(500);
    // rightBackAzimuthMotor.setDesiredOutput(500);
}

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* register subsystems here -------------------------------------------------*/
void registerSentrySubsystems(Drivers *drivers)
{
    // drivers->commandScheduler.registerSubsystem(&sentryDrive);
    // drivers->commandScheduler.registerSubsystem(&turretZero.agitator);
    // drivers->commandScheduler.registerSubsystem(&turretZero.frictionWheels);
    drivers->commandScheduler.registerSubsystem(&turretZero.turretSubsystem);
    // drivers->commandScheduler.registerSubsystem(&turretOne.agitator);
    // drivers->commandScheduler.registerSubsystem(&turretOne.frictionWheels);
    drivers->commandScheduler.registerSubsystem(&turretOne.turretSubsystem);
    drivers->commandScheduler.registerSubsystem(&turretMajor);
    // drivers->commandScheduler.registerSubsystem(&odometrySubsystem);
    // drivers->visionCoprocessor.attachOdometryInterface(&odometrySubsystem);
    // drivers->visionCoprocessor.attachTurretOrientationInterface(&turretZero.turretSubsystem, 0);
    // drivers->visionCoprocessor.attachTurretOrientationInterface(&turretOne.turretSubsystem, 1);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSentryCommands(Drivers *)
{
    // sentryDrive.setDefaultCommand(&chassisAutorotateCommand);
    // TEMP: Setting default command to manual drive
    // sentryDrive.setDefaultCommand(&chassisDriveCommand);
    // turretZero.frictionWheels.setDefaultCommand(&turretZero.spinFrictionWheels);
    // turretOne.frictionWheels.setDefaultCommand(&turretOne.spinFrictionWheels);
    // turretZero.turretSubsystem.setDefaultCommand(&turretZero.turretCVCommand);
    // turretOne.turretSubsystem.setDefaultCommand(&turretOne.turretCVCommand);
    // turretZero.agitator.setDefaultCommand(
    //     &turretZero.rotateAndUnjamAgitatorWithHeatAndCvLimitingWhenCvOnline);
    // turretOne.agitator.setDefaultCommand(
    //     &turretOne.rotateAndUnjamAgitatorWithHeatAndCvLimitingWhenCvOnline);

    
}

/* add any starting commands to the scheduler here --------------------------*/
void startSentryCommands(Drivers *drivers)
{
    // drivers->commandScheduler.addCommand(&imuCalibrateCommand);

    // sentryRequestHandler.attachPauseProjectileLaunchingMessageHandler(
    // // //     pauseProjectileLaunchMessageHandler);
    // // sentryRequestHandler.attachSelectNewRobotMessageHandler(selectNewRobotMessageHandler);
    // sentryRequestHandler.attachTargetNewQuadrantMessageHandler(targetNewQuadrantMessageHandler);
    drivers->refSerial.attachRobotToRobotMessageHandler(
        aruwsrc::communication::serial::SENTRY_REQUEST_ROBOT_ID,
        &sentryRequestHandler);
}

/* register io mappings here ------------------------------------------------*/
void registerSentryIoMappings(Drivers *drivers)
{
    // drivers->commandMapper.addMap(&rightSwitchDown);
    // drivers->commandMapper.addMap(&rightSwitchUp);
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
// imu::ImuCalibrateCommand *getImuCalibrateCommand() { return &sentry_control::imuCalibrateCommand; }
#endif

#endif
