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
#include "tap/drivers.hpp"
#include "tap/motor/double_dji_motor.hpp"
#include "tap/motor/servo.hpp"
#include "tap/communication/sensors/limit_switch/limit_switch_interface.hpp"

#include "aruwsrc/robot/dart/dart_yaw_position_command.hpp"
#include "aruwsrc/control/joint/homing/trigger_homed_joint_subsystem.hpp"
#include "aruwsrc/communication/low_battery_buzzer_command.hpp"
#include "aruwsrc/control/buzzer/buzzer_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/dart/dart_constants.hpp"
#include "aruwsrc/robot/dart/dart_drivers.hpp"
#include "aruwsrc/robot/dart/dart_launcher_subsystem.hpp"
#include "aruwsrc/communication/sensors/beam_break/beam_break.hpp"
#include "aruwsrc/control/joint/homing/trigger/limit_switch_trigger.hpp"
#include "aruwsrc/control/joint/homing/homing_command.hpp"
#include "aruwsrc/robot/dart/dart_yaw_velocity_command.hpp"

#include "dart_close_command.hpp"
#include "dart_open_command.hpp"
#include "dart_pullback_command.hpp"
#include "dart_release_command.hpp"
#include <tap/control/press_command_mapping.hpp>

using namespace tap::control;
using namespace aruwsrc::control;
using namespace tap::communication::serial;
using namespace aruwsrc::dart;
using namespace aruwsrc::robot::dart;
using namespace aruwsrc::communication::sensors;
using namespace aruwsrc::control::joint::homing::trigger;
using namespace aruwsrc::control::joint::homing;
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
tap::motor::DoubleDjiMotor pullMotors(
    drivers(),
    UPPER_PULL_MOTOR_ID,
    LOWER_PULL_MOTOR_ID,
    LAUNCHER_CAN_BUS,
    LAUNCHER_CAN_BUS,
    true,
    true,
    "Upper Motor",
    "Lower Motor");


RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

tap::motor::DjiMotor yawMotor(
    drivers(),
    YAW_MOTOR_ID,
    LAUNCHER_CAN_BUS,
    true,
    "Yaw Motor",
    false
);

aruwsrc::communication::sensors::beam_break::DigitalBeamBreak yawLimitSwitch(
    &drivers()->digital,
    YAW_LIMITSWITCH_PORT,
    false
);

LimitSwitchTrigger yawTrigger(
    &yawLimitSwitch
);

aruwsrc::control::joint::homing::TriggerHomedJointSubsystem yawSubsystem(
    drivers(),
    yawMotor,
    yawTrigger,
    YAW_HOME_CONFIG
);

DartYawVelocityCommand dartYawVelocityCommand(
    drivers(),
    &yawSubsystem,
    Remote::Channel::LEFT_HORIZONTAL
);

DartLauncherSubsystem dartLauncher(drivers(), pullMotors);

DartReleaseCommand dartRelease(dartLauncher, MANUAL_RELEASE_DESIRED_OUTPUT);
DartPullbackCommand dartPullback(dartLauncher, MANUAL_PULLBACK_DESIRED_OUTPUT);

DartOpenCommand servoOpen(dartLauncher);
DartCloseCommand servoClose(dartLauncher);

HomingCommand yawHome(yawSubsystem);

DartYawPositionCommand dartYawPositionCommand(
    drivers(),
    &yawSubsystem,
    0.0f
);

tap::control::PressCommandMapping leftSwitchDown(
    drivers(),
    {&yawHome},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));

tap::control::HoldCommandMapping leftSwitchUp(
    drivers(),
    {&dartYawPositionCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// HoldCommandMapping rightSwitchUp(
//     drivers(),
//     {&dartPullback},
//     RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

// HoldCommandMapping rightSwitchDown(
//     drivers(),
//     {&dartRelease},
//     RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));

// HoldCommandMapping leftSwitchUp(
//     drivers(),
//     {&servoOpen},
//     RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// HoldCommandMapping leftSwitchDown(
//     drivers(),
//     {&servoClose},
//     RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));

void initializeSubsystems() { 
    dartLauncher.initialize(); 
    yawSubsystem.initialize();
}

void registerDartSubsystems(aruwsrc::dart::Drivers* drivers)
{
    drivers->commandScheduler.registerSubsystem(&yawSubsystem);
    drivers->commandScheduler.registerSubsystem(&dartLauncher);
    drivers->digital.configureInputPullMode(
        tap::gpio::Digital::B,
        tap::gpio::Digital::InputPullMode::PullUp);
}

void setDefaultDartCommands(aruwsrc::dart::Drivers*) {
    yawSubsystem.setDefaultCommand(&dartYawVelocityCommand);
}

void startDartCommands(aruwsrc::dart::Drivers*) {}

void registerDartIoMappings(aruwsrc::dart::Drivers* drivers)
{
    // drivers->commandMapper.addMap(&rightSwitchUp);
    // drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchDown);
}

}  // namespace dart_control
namespace aruwsrc::dart
{
void initSubsystemCommands(Drivers* drivers)
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
