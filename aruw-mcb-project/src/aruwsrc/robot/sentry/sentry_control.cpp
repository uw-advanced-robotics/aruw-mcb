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

#include "aruwsrc/communication/mcb-lite/motor/virtual_dji_motor.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "aruwsrc/control/chassis/new_sentry/sentry_manual_drive_command.hpp"
#include "aruwsrc/control/chassis/new-chassis/chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/new-chassis/swerve_wheel.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/sentry/sentry_chassis_constants.hpp"

using namespace aruwsrc::sentry;
using namespace aruwsrc::control;
using namespace aruwsrc::chassis;
using namespace aruwsrc::virtualMCB;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace sentry_control
{
/* define subsystems --------------------------------------------------------*/

VirtualDjiMotor leftFrontDriveMotor(
    drivers(),
    MOTOR2,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->mcbLite),
    aruwsrc::sentry::chassis::leftFrontSwerveConfig.inverted,
    "Left Front Swerve Drive Motor");

VirtualDjiMotor leftFrontAzimuthMotor(
    drivers(),
    MOTOR6,
    // MOTOR5,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->mcbLite),
    aruwsrc::sentry::chassis::leftFrontSwerveConfig.inverted,
    "Left Front Swerve Azimuth Motor");

VirtualDjiMotor rightFrontDriveMotor(
    drivers(),
    MOTOR1,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->mcbLite),
    aruwsrc::sentry::chassis::rightFrontSwerveConfig.inverted,  // TODO: BRUHHH
    "Right Front Swerve Drive Motor");

VirtualDjiMotor rightFrontAzimuthMotor(
    drivers(),
    MOTOR5,
    tap::can::CanBus::CAN_BUS1,
    &(drivers()->mcbLite),
    aruwsrc::sentry::chassis::rightFrontSwerveConfig.inverted,
    "Right Front Swerve Azimuth Motor");

VirtualDjiMotor leftBackDriveMotor(
    drivers(),
    MOTOR3,
    tap::can::CanBus::CAN_BUS2,
    &(drivers()->mcbLite),
    aruwsrc::sentry::chassis::leftBackSwerveConfig.inverted,
    "Left Back Swerve Drive Motor");

VirtualDjiMotor leftBackAzimuthMotor(
    drivers(),
    MOTOR7,
    tap::can::CanBus::CAN_BUS2,
    &(drivers()->mcbLite),
    aruwsrc::sentry::chassis::leftBackSwerveConfig.inverted,
    "Left Back Swerve Azimuth Motor");

VirtualDjiMotor rightBackDriveMotor(
    drivers(),
    MOTOR4,
    tap::can::CanBus::CAN_BUS2,
    &(drivers()->mcbLite),
    aruwsrc::sentry::chassis::rightBackSwerveConfig.inverted,
    "Right Back Swerve Drive Motor");

VirtualDjiMotor rightBackAzimuthMotor(
    drivers(),
    MOTOR8,
    tap::can::CanBus::CAN_BUS2,
    &(drivers()->mcbLite),
    aruwsrc::sentry::chassis::rightBackSwerveConfig.inverted,
    "Right Back Swerve Azimuth Motor");

// these four swerve modules will later be passed into SwerveChassisSubsystem
aruwsrc::chassis::SwerveWheel leftFrontSwerveModule(
    leftFrontDriveMotor,
    leftFrontAzimuthMotor,
    aruwsrc::sentry::chassis::leftFrontSwerveConfig,
    aruwsrc::sentry::chassis::leftFrontSwerveAzimuthConfig,
    aruwsrc::sentry::chassis::leftFrontSwerveConfig.velocityPidConfig,
    aruwsrc::sentry::chassis::leftFrontSwerveAzimuthConfig.azimuthPidConfig
    );

aruwsrc::chassis::SwerveWheel rightFrontSwerveModule(
    rightFrontDriveMotor,
    rightFrontAzimuthMotor,
    aruwsrc::sentry::chassis::rightFrontSwerveConfig,
    aruwsrc::sentry::chassis::rightFrontSwerveAzimuthConfig,
    aruwsrc::sentry::chassis::rightFrontSwerveConfig.velocityPidConfig,
    aruwsrc::sentry::chassis::rightFrontSwerveAzimuthConfig.azimuthPidConfig);

aruwsrc::chassis::SwerveWheel leftBackSwerveModule(
    leftBackDriveMotor,
    leftBackAzimuthMotor,
    aruwsrc::sentry::chassis::leftBackSwerveConfig,
    aruwsrc::sentry::chassis::leftBackSwerveAzimuthConfig,
    aruwsrc::sentry::chassis::leftBackSwerveConfig.velocityPidConfig,
    aruwsrc::sentry::chassis::leftBackSwerveAzimuthConfig.azimuthPidConfig
    );

aruwsrc::chassis::SwerveWheel rightBackSwerveModule(
    rightBackDriveMotor,
    rightBackAzimuthMotor,
    aruwsrc::sentry::chassis::rightBackSwerveConfig,
    aruwsrc::sentry::chassis::rightBackSwerveAzimuthConfig,
    aruwsrc::sentry::chassis::rightBackSwerveConfig.velocityPidConfig,
    aruwsrc::sentry::chassis::rightBackSwerveAzimuthConfig.azimuthPidConfig
    );

aruwsrc::chassis::ChassisSubsystem chassis(
    &drivers()->mcbLite.currentSensor,
    {&leftFrontSwerveModule,
    &rightFrontSwerveModule,
    &leftBackSwerveModule,
    &rightBackSwerveModule},
    aruwsrc::sentry::chassis::SWERVE_FORWARD_MATRIX);

/* define commands ----------------------------------------------------------*/
aruwsrc::control::sentry::SentryManualDriveCommand chassisDriveCommand(
    drivers(),
    &(drivers()->controlOperatorInterface),
    &chassis);

/* define command mappings --------------------------------------------------*/

/* initialize subsystems ----------------------------------------------------*/

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

void initializeSubsystems() { chassis.initialize(); }

// note: some stubs commented out because CI screams about unused parameters
/* register subsystems here -------------------------------------------------*/
void registerSentrySubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassis);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSentryCommands(Drivers *) { chassis.setDefaultCommand(&chassisDriveCommand); }

/* add any starting commands to the scheduler here --------------------------*/
void startSentryCommands(Drivers *drivers) { drivers = drivers; }

/* register io mappings here ------------------------------------------------*/
void registerSentryIoMappings(Drivers *drivers) { drivers = drivers; }
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
