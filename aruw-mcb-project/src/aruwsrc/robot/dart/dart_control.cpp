/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#if defined(TARGET_DART)

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"

#include "aruwsrc/control/pneumatic/gpio_double_solenoid.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/dart/dart_constants.hpp"
#include "aruwsrc/robot/dart/dart_drivers.hpp"
#include "aruwsrc/robot/dart/launcher/launcher_pull_subsystem.hpp"
#include "aruwsrc/robot/dart/launcher/launcher_release_subsystem.hpp"
#include "aruwsrc/robot/dart/launcher/manual_launcher_pull_command.hpp"
#include "aruwsrc/robot/dart/launcher/manual_launcher_release_command.hpp"
#include "aruwsrc/robot/dart/loader/loader_subsystem.hpp"
#include "aruwsrc/robot/dart/loader/manual_loader_command.hpp"
#include "aruwsrc/robot/dart/pivot/manual_pivot_command.hpp"
#include "aruwsrc/robot/dart/pivot/pivot_subsystem.hpp"
#include "aruwsrc/control/bounded-subsystem/trigger/motor_stall_trigger.hpp"

using namespace aruwsrc::control::turret;
using namespace tap::control;
using namespace aruwsrc::control;
using namespace tap::communication::serial;
using namespace aruwsrc::robot::dart;
using namespace aruwsrc::dart;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;
namespace dart_control
{
/* define subsystems ----------------------------------------------*/
tap::motor::DjiMotor launcherPullMotor1(
    drivers(),
    LAUNCHER_PULL_MOTOR_1_ID,
    CAN_BUS_MOTORS,
    false,
    "Pull Motor 1");

tap::motor::DjiMotor launcherPullMotor2(
    drivers(),
    LAUNCHER_PULL_MOTOR_2_ID,
    CAN_BUS_MOTORS,
    false,
    "Pull Motor 2");

tap::motor::DjiMotor launcherEncoderMotor(
    drivers(),
    LAUNCHER_ENCODER_MOTOR_ID,
    CAN_BUS_MOTORS,
    false,
    "Pull Motor 2");

aruwsrc::control::pneumatic::GpioDoubleSolenoid launcherLinearActuator(
    drivers(),
    LAUNCHER_ACTUATOR_LOCK_PIN,
    LAUNCHER_ACTUATOR_RELEASE_PIN);

tap::motor::DjiMotor pivotMotor(drivers(), PIVOT_MOTOR_ID, CAN_BUS_MOTORS, false, "Pivot Motor");

tap::motor::DjiMotor pivotDeadMotor(
    drivers(),
    PIVOT_DEAD_MOTOR_ID,
    CAN_BUS_MOTORS,
    false,
    "Pivot Dead Motor");

tap::motor::DjiMotor loaderTopMotor(
    drivers(),
    LOADER_TOP_MOTOR_ID,
    CAN_BUS_MOTORS,
    false,
    "Loader Top Motor");

tap::motor::DjiMotor loaderMiddleMotor(
    drivers(),
    LOADER_MIDDLE_MOTOR_ID,
    CAN_BUS_MOTORS,
    false,
    "Loader Middle Motor");

tap::motor::DjiMotor loaderBottomMotor(
    drivers(),
    LOADER_BOTTOM_MOTOR_ID,
    CAN_BUS_MOTORS,
    false,
    "Loader Bottom Motor");

/** Subsystems here */

LauncherPullSubsystem launcherPullSubsystem(
    drivers(),
    &launcherPullMotor1,
    &launcherPullMotor2,
    &launcherEncoderMotor,
    launcherPullPID);

LauncherReleaseSubsystem launcherReleaseSubsystem(
    drivers(),
    &launcherLinearActuator);



PivotSubsystem pivotSubsystem(
    drivers(),
    &pivotMotor,
    &pivotDeadMotor,
    pivotPID);

LoaderSubsystem loaderSubsystem(
    drivers(),
    &loaderTopMotor,
    &loaderMiddleMotor,
    &loaderBottomMotor,
    loaderPID);

/* All of the commands under here*/

ManualLauncherPullCommand manualLauncherPullCommand(
    drivers(),
    &launcherPullSubsystem,
    Remote::Channel::WHEEL);

ManualLauncherReleaseCommand manualLauncherReleaseCommand(
    drivers(),
    &launcherReleaseSubsystem,
    Remote::Switch::RIGHT_SWITCH);

ManualLoaderCommand manualLoaderCommand(
    drivers(),
    &loaderSubsystem,
    Remote::Switch::LEFT_SWITCH,
    Remote::Channel::LEFT_VERTICAL);

ManualPivotCommand manualPivotCommand(
    drivers(),
    &pivotSubsystem,
    Remote::Channel::RIGHT_HORIZONTAL);

/** Do mapping here */



RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

void initializeSubsystems()
{
    launcherPullSubsystem.initialize();
    launcherReleaseSubsystem.initialize();
    pivotSubsystem.initialize();
    loaderSubsystem.initialize();
}

void registerDartSubsystems(Drivers* drivers)
{
    drivers->commandScheduler.registerSubsystem(&launcherPullSubsystem);
    drivers->commandScheduler.registerSubsystem(&launcherReleaseSubsystem);
    drivers->commandScheduler.registerSubsystem(&pivotSubsystem);
    drivers->commandScheduler.registerSubsystem(&loaderSubsystem);
}

void setDefaultDartCommands(Drivers* drivers)
{
    pivotSubsystem.setDefaultCommand(&manualPivotCommand);
    loaderSubsystem.setDefaultCommand(&manualLoaderCommand);
    launcherReleaseSubsystem.setDefaultCommand(&manualLauncherReleaseCommand);
    launcherPullSubsystem.setDefaultCommand(&manualLauncherPullCommand);
}

void startDartCommands(Drivers* drivers)
{
}

void registerDartIoMappings(Drivers* drivers)
{ 
}

}  // namespace dart_control

namespace aruwsrc::dart
{
void initSubsystemCommands(aruwsrc::dart::Drivers* drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &dart_control::remoteSafeDisconnectFunction);
    dart_control::initializeSubsystems();
    dart_control::registerDartSubsystems(drivers);
    dart_control::setDefaultDartCommands(drivers);
    dart_control::startDartCommands(drivers);
    dart_control::registerDartIoMappings(drivers);
}
}  // namespace aruwsrc::dart

#endif
