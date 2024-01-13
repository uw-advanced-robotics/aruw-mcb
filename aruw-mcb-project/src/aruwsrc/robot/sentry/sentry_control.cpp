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
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/double_dji_motor.hpp"

#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/yaw_turret_subsystem.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/sentry/sentry_control_operator_interface.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_minor_subsystem.hpp"
#include "aruwsrc/robot/sentry/turret_major_control_command.hpp"
#include "aruwsrc/robot/sentry/turret_minor_control_command.hpp"

using namespace tap::algorithms;
using namespace tap::control;
using namespace tap::communication::serial;
using namespace aruwsrc::sentry;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::control::sentry;
using namespace aruwsrc::control::turret::sentry;
using namespace aruwsrc::control::turret::algorithms;

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

/* define subsystems --------------------------------------------------------*/
YawTurretSubsystem turretMajor(*drivers(), turretMajorYawMotor, turretMajor::YAW_MOTOR_CONFIG);

SentryTurretMinorSubsystem turretLeft(
    *drivers(),
    turretLeftMotors.pitchMotor,
    turretLeftMotors.yawMotor,
    turretLeftMotors.yawMotorConfig,
    turretLeftMotors.pitchMotorConfig,
    &drivers()->turretMCBCanCommBus2,  // @todo: figure out how to put this in config
    turretLeft::turretID);

SentryTurretMinorSubsystem turretRight(
    *drivers(),
    turretRightMotors.pitchMotor,
    turretRightMotors.yawMotor,
    turretRightMotors.yawMotorConfig,
    turretRightMotors.pitchMotorConfig,
    &drivers()->turretMCBCanCommBus1,  // @todo: figure out how to put this in config
    turretRight::turretID);

/* define controllers --------------------------------------------------------*/

// @note: this should be replaced with a world-relative controller, just for manual testing now
ChassisFrameYawTurretController majorController(
    turretMajor.getMotor(),
    turretMajor::chassisFrameController::YAW_POS_PID_CONFIG);

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

/* define commands ----------------------------------------------------------*/
TurretMajorSentryControlCommand majorManualCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    turretMajor,
    majorController,
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

/* define command mappings --------------------------------------------------*/
HoldCommandMapping manualRightSwitchDown(
    drivers(),
    {
        &majorManualCommand
        /*
        // @note: minor commands commented out so I can test major control first
        ,
         &turretLeftManualCommand,
         &turretRightManualCommand
        */
    },
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::DOWN));

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    turretMajor.initialize();

    // @note: initialization of controllers usually handled by imu calibrate, so
    // move this stuff into there once imu calibrate integrated
    majorController.initialize();

    turretLeftControllers.pitchController.initialize();
    turretLeftControllers.yawController.initialize();

    turretRightControllers.pitchController.initialize();
    turretRightControllers.yawController.initialize();

    turretLeft.initialize();
    turretRight.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerSentrySubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&turretMajor);
    drivers->commandScheduler.registerSubsystem(&turretLeft);
    drivers->commandScheduler.registerSubsystem(&turretRight);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSentryCommands(Drivers *)
{
    turretMajor.setDefaultCommand(&majorManualCommand);
    turretLeft.setDefaultCommand(&turretLeftManualCommand);
    turretRight.setDefaultCommand(&turretRightManualCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSentryCommands(Drivers *drivers) { drivers = drivers; }  // @todo: imu calibrate command

/* register io mappings here ------------------------------------------------*/
void registerSentryIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(&manualRightSwitchDown);
}
}  // namespace sentry_control

namespace aruwsrc::sentry
{
void initSubsystemCommands(aruwsrc::sentry::Drivers *drivers)
{
    sentry_control::initializeSubsystems();
    sentry_control::registerSentrySubsystems(drivers);
    sentry_control::setDefaultSentryCommands(drivers);
    sentry_control::startSentryCommands(drivers);
    sentry_control::registerSentryIoMappings(drivers);
}
}  // namespace aruwsrc::sentry

#endif
