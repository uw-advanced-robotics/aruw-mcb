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
#include "aruwsrc/control/joint/homing/homeable_subsystem_interface.hpp"
#if defined(TARGET_DART)
#include <memory>

#include "tap/communication/sensors/limit_switch/limit_switch_interface.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/remote_map_state.hpp"
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
    yawSubsystem,
    &drivers()->controlOperatorInterface);
// grab the string and pullback to setpoint
SequentialCommand<2> pullBackCommand(&servoClose, &dartPullback);

// release the string to let the dart go, then go to reload position
SequentialCommand<3> releaseDartAndReload(&servoOpen, &dartGrab, &rotateMagazine);

SequentialCommand<2> homeAll(&pullMotorHome, &yawHomeCommand);

// Left Up + Right Down -> Servo Open
RemoteMapState openServoRemoteMapState =
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::DOWN);
auto openServoMapping = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command*>{&servoOpen},
    &openServoRemoteMapState);

// Left down + Right Down -> Servo Close
RemoteMapState closeServoRemoteMapState =
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::DOWN);
auto closeServoMapping = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command*>{&servoClose},
    &closeServoRemoteMapState);

// Left Mid + Right Up -> Home Pullback
auto homePullbackRemoteMapState = RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::UP);
auto homePullbackCommand = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command*>{&pullMotorHome},
    &homePullbackRemoteMapState);

// Left Mid + Right Down -> Pullback Dart
auto pullbackRemoteMapState = RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::DOWN);
auto pullbackCommand = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command*>{&dartPullback},
    &pullbackRemoteMapState);

auto rightMidLeftDownRms = RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::UP);
auto rightMidLeftDown = std::make_unique<PressCommandMapping>(
    drivers(),
    std::vector<Command*>{&rotateMagazine},
    &rightMidLeftDownRms);

// Left Down + Right Up -> Home Yaw
auto homeYawRemoteMapState = RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::UP);
auto homeYawCommand = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command*>{&yawHomeCommand},
    &homeYawRemoteMapState);

void initializeSubsystems()
{
    dartServo.initialize();
    pullMotorSubsystem.initialize();
    dartReloader.initialize();
    yawSubsystem.initialize();
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
    drivers->commandMapper.addMap(std::move(openServoMapping));
    drivers->commandMapper.addMap(std::move(closeServoMapping));
    // drivers->commandMapper.addMap(std::move(homePullbackCommand));
    // drivers->commandMapper.addMap(std::move(pullbackCommand));
    //  TODO: uncomment when dart squad reassembles this, currently not attached and dont wanna risk
    //  robot damage
    // drivers->commandMapper.addMap(std::move(rightMidLeftDown));
    drivers->commandMapper.addMap(std::move(homeYawCommand));
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
