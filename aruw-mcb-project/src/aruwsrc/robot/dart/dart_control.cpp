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

#include "tap/communication/sensors/limit_switch/limit_switch_interface.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/sequential_command.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/double_dji_motor.hpp"
#include "tap/motor/servo.hpp"

#include "aruwsrc/communication/low_battery_buzzer_command.hpp"
#include "aruwsrc/communication/sensors/beam_break/beam_break.hpp"
#include "aruwsrc/control/buzzer/buzzer_subsystem.hpp"
#include "aruwsrc/control/joint/homing/homing_command.hpp"
#include "aruwsrc/control/joint/homing/trigger/limit_switch_trigger.hpp"
#include "aruwsrc/control/joint/homing/trigger_homed_joint_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/dart/dart_constants.hpp"
#include "aruwsrc/robot/dart/dart_control_operator_interface.hpp"
#include "aruwsrc/robot/dart/dart_drivers.hpp"
#include "aruwsrc/robot/dart/dart_manual_pullback_setpoint_command.hpp"
#include "aruwsrc/robot/dart/dart_reloader_subsystem.hpp"
#include "aruwsrc/robot/dart/dart_servo.hpp"
#include "aruwsrc/robot/dart/dart_yaw_position_command.hpp"
#include "aruwsrc/robot/dart/dart_yaw_velocity_command.hpp"

#include "dart_close_command.hpp"
#include "dart_constants.hpp"
#include "dart_open_command.hpp"
#include "dart_release_command.hpp"
#include "dart_setpoint_command.hpp"
#include "rotate_magazine_command.hpp"

using namespace tap::control;
using namespace aruwsrc::control;
using namespace tap::communication::serial;
using namespace aruwsrc::dart;
using namespace aruwsrc::control::joint;
using namespace aruwsrc::control::joint::homing;
using namespace aruwsrc::control::joint::homing::trigger;
using namespace aruwsrc::communication::sensors;
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

aruwsrc::communication::sensors::beam_break::DigitalBeamBreak limitSwitch(
    &(drivers()->digital),
    LIMITSWITCH_PORT,
    false);

aruwsrc::control::joint::homing::trigger::LimitSwitchTrigger limit(&limitSwitch);
aruwsrc::control::joint::homing::TriggerHomedJointSubsystem pullMotorSubsystem(
    drivers(),
    pullMotors,
    limit,
    PULL_MOTOR_CONFIG);

aruwsrc::communication::sensors::beam_break::DigitalBeamBreak yawLimitSwitch(
    &(drivers()->digital),
    YAW_LIMITSWITCH_PORT,
    false);
LimitSwitchTrigger yawTrigger(&yawLimitSwitch);
tap::motor::DjiMotor yawMotor(drivers(), YAW_MOTOR_ID, LAUNCHER_CAN_BUS, true, "Yaw Motor");

aruwsrc::control::joint::homing::TriggerHomedJointSubsystem yawSubsystem(
    drivers(),
    yawMotor,
    yawTrigger,
    YAW_HOME_CONFIG);

HomingCommand pullMotorHomeCommand(pullMotorSubsystem);
DartManualPullbackSetpointCommand manualPullbackCommand(
    pullMotorSubsystem,
    MANUAL_PULLBACK_SPEED_MULTIPLIER,
    &drivers()->controlOperatorInterface);

tap::motor::DjiMotor reloaderMotor(
    drivers(),
    RELOADER_MOTOR_ID,
    RELOADER_CAN_BUS,
    false,
    "Reloader Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M2006 / 6.25);

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

DartReloaderSubsystem dartReloader(drivers(), reloaderMotor);

RotateMagazineCommand rotateMagazine(dartReloader);

DartServo dartServo(drivers());
DartSetpointCommand dartPullback(pullMotorSubsystem, PULLBACK_PULL_POSITION);
DartSetpointCommand dartGrab(pullMotorSubsystem, GRAB_POSITION);
DartOpenCommand servoOpen(dartServo);
DartCloseCommand servoClose(dartServo);
HomingCommand pullMotorHome(pullMotorSubsystem);
HomingCommand yawHomeCommand(yawSubsystem);

aruwsrc::robot::dart::DartYawPositionCommand dartYawPositionCommand(drivers(), &yawSubsystem, 0.0f);

// yaw manual velocity control, LEFT_X
aruwsrc::robot::dart::DartYawVelocityCommand dartYawVelocityCommand(
    drivers(),
    yawSubsystem,
    Remote::Channel::LEFT_HORIZONTAL);
// grab the string and pullback to setpoint
SequentialCommand<2> pullBackCommand(std::array<Command*, 2>{{&servoClose, &dartPullback}});

// release the string to let the dart go, then go to reload position
SequentialCommand<3> releaseDartAndReload(
    std::array<Command*, 3>{{&servoOpen, &dartGrab, &rotateMagazine}});

SequentialCommand<2> homeAll(std::array<Command*, 2>{{&pullMotorHome, &yawHomeCommand}});

// Left Up + Right Up -> Servo Open
HoldCommandMapping openServoMapping(
    drivers(),
    {&servoOpen},
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::UP));

// Left Up + Right Down -> Servo Close
HoldCommandMapping closeServoMapping(
    drivers(),
    {&servoClose},
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::DOWN));
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
//     RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::UP));

// Left Mid + Right Up -> Home Pullback
HoldCommandMapping homePullbackMapping(
    drivers(),
    {&pullMotorHome},
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::UP));

// Left Mid + Right Down -> Pullback Dart
HoldCommandMapping pullbackMapping(
    drivers(),
    {&dartPullback},
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::DOWN));

PressCommandMapping rightMidLeftDown(
    drivers(),
    {&rotateMagazine},
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::MID));

// Left Down + Right Up -> Home Yaw
HoldCommandMapping homeYawMapping(
    drivers(),
    {&yawHomeCommand},
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::UP));

void initializeSubsystems()
{
    dartServo.initialize();
    pullMotorSubsystem.initialize();
    dartReloader.initialize();
}

void registerDartSubsystems(aruwsrc::dart::Drivers* drivers)
{
    drivers->commandScheduler.registerSubsystem(&yawSubsystem);
    drivers->commandScheduler.registerSubsystem(&dartServo);
    drivers->commandScheduler.registerSubsystem(&pullMotorSubsystem);
    drivers->commandScheduler.registerSubsystem(&dartReloader);
    drivers->digital.configureInputPullMode(
        tap::gpio::Digital::B,
        tap::gpio::Digital::InputPullMode::PullUp);
}

void setDefaultDartCommands(aruwsrc::dart::Drivers*)
{
    pullMotorSubsystem.setDefaultCommand(&manualPullbackCommand);

    yawSubsystem.setDefaultCommand(&dartYawVelocityCommand);
}

void startDartCommands(aruwsrc::dart::Drivers*) {}

void registerDartIoMappings(aruwsrc::dart::Drivers* drivers)
{
    drivers->commandMapper.addMap(&openServoMapping);
    drivers->commandMapper.addMap(&closeServoMapping);
    drivers->commandMapper.addMap(&homePullbackMapping);
    drivers->commandMapper.addMap(&pullbackMapping);
    drivers->commandMapper.addMap(&rightMidLeftDown);
    drivers->commandMapper.addMap(&homeYawMapping);
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
