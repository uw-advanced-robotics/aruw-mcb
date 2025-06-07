/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#if defined(TARGET_ENGINEER)

#include "tap/communication/gpio/digital.hpp"
#include "tap/communication/sensors/encoder/can_encoder/can_encoder.hpp"
#include "tap/communication/sensors/limit_switch/limit_switch_interface.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/command_scheduler.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"

#include "aruwsrc/communication/sensors/beam_break/beam_break.hpp"
#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/communication/sensors/voltage/fake_voltage_sensor.hpp"
#include "aruwsrc/control/bounded-subsystem/homing_command.hpp"
#include "aruwsrc/control/bounded-subsystem/trigger/limit_switch_trigger.hpp"
#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "aruwsrc/control/chassis/mecanum_chassis_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/engineer/cube_lift/cube_storage_subsystem.hpp"
#include "aruwsrc/robot/engineer/digital_out_command.hpp"
#include "aruwsrc/robot/engineer/digital_out_subsystem.hpp"
#include "aruwsrc/robot/engineer/engineer_drivers.hpp"
#include "aruwsrc/robot/engineer/engineer_gantry_constants.hpp"
#include "aruwsrc/robot/engineer/engineer_lift_constants.hpp"
#include "aruwsrc/robot/engineer/engineer_wrist_constants.hpp"
#include "aruwsrc/robot/engineer/gantry/gantry_extension_subsystem.hpp"
#include "aruwsrc/robot/engineer/gantry/gantry_lift_subsystem.hpp"
#include "aruwsrc/robot/engineer/joint_subsystem.hpp"
#include "aruwsrc/robot/engineer/setpoint_move_manual_command.hpp"
#include "aruwsrc/robot/engineer/setpoint_move_position_command.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_controller_command.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_setpoints_command.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_subsystem.hpp"

using namespace tap::gpio;
using tap::communication::serial::Remote;
using tap::control::CommandMapper;
using namespace aruwsrc::control::engineer;
using namespace aruwsrc::engineer;
using namespace aruwsrc::engineer::gantry;
using namespace aruwsrc::engineer::lift;
using namespace aruwsrc::engineer::wrist;
using namespace tap::control;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace aruwsrc
{
namespace control
{
aruwsrc::communication::sensors::voltage::FakeVoltageSensor voltageSensor;

tap::motor::DjiMotor leftFrontChassisMotor(
    drivers(),
    aruwsrc::chassis::LEFT_FRONT_MOTOR_ID,
    aruwsrc::chassis::CAN_BUS_MOTORS,
    false,
    "Left Front Chassis Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor leftBackChassisMotor(
    drivers(),
    aruwsrc::chassis::LEFT_BACK_MOTOR_ID,
    aruwsrc::chassis::CAN_BUS_MOTORS,
    false,
    "Left Back Chassis Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor rightFrontChassisMotor(
    drivers(),
    aruwsrc::chassis::RIGHT_FRONT_MOTOR_ID,
    aruwsrc::chassis::CAN_BUS_MOTORS,
    false,
    "Right Front Chassis Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor rightBackChassisMotor(
    drivers(),
    aruwsrc::chassis::RIGHT_BACK_MOTOR_ID,
    aruwsrc::chassis::CAN_BUS_MOTORS,
    false,
    "Right Back Chassis Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::communication::sensors::current::AnalogCurrentSensor currentSensor(
    {&drivers()->analog,
     aruwsrc::chassis::CURRENT_SENSOR_PIN,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_MV_PER_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_ZERO_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA});

tap::motor::DjiMotor cubeLiftMotor(
    drivers(),
    CUBE_LIFT_MOTOR_ID,
    LIFT_MOTOR_CAN_BUS,
    true,
    "Lifting Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

aruwsrc::communication::sensors::beam_break::DigitalBeamBreak cubeLiftLimit(
    &(drivers()->digital),
    CUBELIFT_LIMITSWITCH_PORT,
    true);

LimitSwitchTrigger cubeLiftTrigger(&cubeLiftLimit);

tap::motor::DjiMotor wristRollMotor(
    drivers(),
    aruwsrc::engineer::WRIST_ROLL_MOTOR_ID,
    aruwsrc::engineer::CAN_BUS_WRIST,
    false,
    "Wrist Roll Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor wristLeftMotor(
    drivers(),
    aruwsrc::engineer::WRIST_LEFT_MOTOR_ID,
    aruwsrc::engineer::CAN_BUS_WRIST,
    false,
    "Wrist Left Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor wristRightMotor(
    drivers(),
    aruwsrc::engineer::WRIST_RIGHT_MOTOR_ID,
    aruwsrc::engineer::CAN_BUS_WRIST,
    false,
    "Wrist Right Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::encoder::CanEncoder wristPitchEncoder(
    drivers(),
    aruwsrc::engineer::WRIST_PITCH_ENCODER_ID,
    aruwsrc::chassis::CAN_BUS_MOTORS,
    false,
    1,
    WRIST_HOME_PITCH * 4096.0f / (M_PI * 2));

tap::encoder::CanEncoder wristYawEncoder(
    drivers(),
    aruwsrc::engineer::WRIST_YAW_ENCODER_ID,
    aruwsrc::chassis::CAN_BUS_MOTORS,
    false,
    1,
    WRIST_HOME_YAW * 4096.0f / (M_PI * 2));

tap::motor::DjiMotor gantryLiftLeftMotor(
    drivers(),
    aruwsrc::engineer::GANTRY_LIFT_LEFT_MOTOR_ID,
    aruwsrc::engineer::CAN_BUS_GANTRY,
    true,
    "Gantry Lift Left Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor gantryLiftRightMotor(
    drivers(),
    aruwsrc::engineer::GANTRY_LIFT_RIGHT_MOTOR_ID,
    aruwsrc::engineer::CAN_BUS_GANTRY,
    false,
    "Gantry Lift Right Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

aruwsrc::communication::sensors::beam_break::DigitalBeamBreak gantryLiftLimit(
    &drivers()->digital,
    aruwsrc::engineer::GANTRY_LIFT_LIMIT_SWITCH_PIN,
    true);

LimitSwitchTrigger gantryLiftTrigger(&gantryLiftLimit);

tap::motor::DjiMotor gantryExtensionMotor(
    drivers(),
    aruwsrc::engineer::GANTRY_EXTENSION_MOTOR_ID,
    aruwsrc::engineer::CAN_BUS_GANTRY,
    true,
    "Gantry Extension Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

aruwsrc::communication::sensors::beam_break::DigitalBeamBreak gantryExtensionLimit(
    &drivers()->digital,
    aruwsrc::engineer::GANTRY_EXTENSION_LIMIT_SWITCH_PIN,
    true);

LimitSwitchTrigger gantryExtensionTrigger(&gantryExtensionLimit);

/* define subsystems --------------------------------------------------------*/
aruwsrc::chassis::MecanumChassisSubsystem chassis(
    drivers(),
    &currentSensor,
    &voltageSensor,
    leftFrontChassisMotor,
    leftBackChassisMotor,
    rightFrontChassisMotor,
    rightBackChassisMotor,
    aruwsrc::chassis::WHEEL_VELOCITY_PID_CONFIG);

CubeStorageSubsystem cubeLift(
    drivers(),
    cubeLiftMotor,
    LIFT_MOTOR_PID_CONFIG,
    LIFT_HOMING_PID_CONFIG,
    cubeLiftTrigger,
    ONE_CUBE_SETPOINT,
    MM_PER_REVOLUTION);

WristSubsystem wristSubsystem(
    drivers(),
    wristLeftMotor,
    wristRightMotor,
    wristPitchEncoder,
    wristYawEncoder,
    aruwsrc::engineer::WRIST_PITCH_CONFIG,
    aruwsrc::engineer::WRIST_YAW_CONFIG,
    aruwsrc::engineer::WRIST_MIN_PITCH,
    aruwsrc::engineer::WRIST_MAX_PITCH,
    aruwsrc::engineer::WRIST_MIN_YAW,
    aruwsrc::engineer::WRIST_MAX_YAW,
    aruwsrc::engineer::WRIST_RATIO);

GantryLiftSubsystem gantryLiftSubsystem(
    drivers(),
    gantryLiftLeftMotor,
    gantryLiftRightMotor,
    aruwsrc::engineer::GANTRY_LIFT_POS_CONFIG,
    aruwsrc::engineer::GANTRY_LIFT_BALANCE_CONFIG,
    gantryLiftTrigger,
    GANTRY_LIFT_RADIUS,
    GANTRY_LIFT_LOWER_BOUND,
    GANTRY_LIFT_UPPER_BOUND,
    GANTRY_LIFT_HOME,
    GANTRY_LIFT_KS,
    GANTRY_LIFT_EPSILON);

GantryExtensionSubsystem gantryExtensionSubsystem(
    drivers(),
    gantryExtensionMotor,
    aruwsrc::engineer::GANTRY_EXTENSION_CONFIG,
    gantryExtensionTrigger,
    GANTRY_EXTENSION_RADIUS,
    GANTRY_EXTENSION_LOWER_BOUND,
    GANTRY_EXTENSION_UPPER_BOUND,
    GANTRY_EXTENSION_HOME,
    GANTRY_EXTENSION_KS,
    GANTRY_EXTENSION_EPSILON);

JointSubsystem wristRollSubsystem(drivers(), wristRollMotor, aruwsrc::engineer::WRIST_ROLL_CONFIG);

aruwsrc::engineer::DigitalOutSubsystem suckSubsystem(
    drivers(),
    drivers()->digital,
    tap::gpio::Digital::OutputPin::Y,
    true);

aruwsrc::engineer::DigitalOutSubsystem releaseSubsystem(
    drivers(),
    drivers()->digital,
    tap::gpio::Digital::OutputPin::Z);

/* define commands ----------------------------------------------------------*/
HomingCommand cubeLiftHome(cubeLift);
HomingCommand gantryLiftHome(gantryLiftSubsystem);
HomingCommand gantryExtensionHome(gantryExtensionSubsystem);

SetpointMoveManualCommand cubeManualControl(
    cubeLift,
    &drivers()->controlOperatorInterface,
    CUBE_LIFT_MOVE_SPEED,
    SetpointType::CUBE_LIFT);

SetpointMoveManualCommand gantryLiftManualControl(
    gantryLiftSubsystem,
    &drivers()->controlOperatorInterface,
    GANTRY_LIFT_MOVE_SPEED,
    SetpointType::GANTRY_LIFT);

SetpointMoveManualCommand gantryExtensionManualControl(
    gantryExtensionSubsystem,
    &drivers()->controlOperatorInterface,
    GANTRY_EXTENSION_MOVE_SPEED,
    SetpointType::GANTRY_EXTENSION);

SetpointMovePositionCommand oneCubePosition(cubeLift, ONE_CUBE_SETPOINT);
SetpointMovePositionCommand twoCubePosition(cubeLift, TWO_CUBE_SETPOINT);
SetpointMovePositionCommand threeCubePosition(cubeLift, THREE_CUBE_SETPOINT);

aruwsrc::chassis::ChassisDriveCommand chassisDriveCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis);

WristControllerCommand wristControllerCommand(
    wristRollSubsystem,
    wristSubsystem,
    &drivers()->controlOperatorInterface,
    aruwsrc::engineer::WRIST_ROLL_SCALING_FACTOR,
    aruwsrc::engineer::WRIST_PITCH_SCALING_FACTOR,
    aruwsrc::engineer::WRIST_YAW_SCALING_FACTOR);

WristSetpointsCommand wristFoldInCommand(
    wristSubsystem,
    {aruwsrc::engineer::WRIST_BOTTOM_SETPOINT,
     aruwsrc::engineer::WRIST_TOP_SETPOINT,
     aruwsrc::engineer::WRIST_IN_SETPOINT});

WristSetpointsCommand wristFoldOutCommand(
    wristSubsystem,
    {aruwsrc::engineer::WRIST_TOP_SETPOINT,
     aruwsrc::engineer::WRIST_BOTTOM_SETPOINT,
     aruwsrc::engineer::WRIST_OUT_SETPOINT});

// todo

aruwsrc::engineer::DigitalOutCommand suckOffCommand(suckSubsystem, false);
aruwsrc::engineer::DigitalOutCommand suckOnCommand(suckSubsystem, true);
aruwsrc::engineer::DigitalOutCommand releaseOffCommand(releaseSubsystem, false);
aruwsrc::engineer::DigitalOutCommand releaseOnCommand(releaseSubsystem, true);

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

tap::control::PressCommandMapping rightUp(
    drivers(),
    {&cubeLiftHome, &gantryLiftHome, &gantryExtensionHome},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

tap::control::HoldCommandMapping rightDown(
    drivers(),
    {&suckOnCommand, &releaseOnCommand},
    tap::control::RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));

tap::control::HoldCommandMapping oneCube(
    drivers(),
    {&oneCubePosition},
    RemoteMapState({Remote::Key::Z}));  // todo

tap::control::HoldCommandMapping twoCube(
    drivers(),
    {&twoCubePosition},
    RemoteMapState({Remote::Key::X}));

tap::control::PressCommandMapping threeCube(
    drivers(),
    {&threeCubePosition},
    RemoteMapState({Remote::Key::C}));

tap::control::PressCommandMapping wristFoldIn(
    drivers(),
    {&wristFoldInCommand},
    RemoteMapState({Remote::Key::V}));

tap::control::PressCommandMapping wristFoldOut(
    drivers(),
    {&wristFoldOutCommand},
    RemoteMapState({Remote::Key::B}));

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    chassis.initialize();
    gantryLiftSubsystem.initialize();
    gantryExtensionSubsystem.initialize();
    wristRollSubsystem.initialize();
    wristSubsystem.initialize();
    cubeLift.initialize();
    suckSubsystem.initialize();
    releaseSubsystem.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerEngineerSubsystems(aruwsrc::engineer::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&gantryLiftSubsystem);
    drivers->commandScheduler.registerSubsystem(&gantryExtensionSubsystem);
    drivers->commandScheduler.registerSubsystem(&wristRollSubsystem);
    drivers->commandScheduler.registerSubsystem(&wristSubsystem);
    drivers->commandScheduler.registerSubsystem(&cubeLift);
    drivers->commandScheduler.registerSubsystem(&suckSubsystem);
    drivers->commandScheduler.registerSubsystem(&releaseSubsystem);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultEngineerCommands(aruwsrc::engineer::Drivers *)
{
    chassis.setDefaultCommand(&chassisDriveCommand);
    gantryLiftSubsystem.setDefaultCommand(&gantryLiftManualControl);
    gantryExtensionSubsystem.setDefaultCommand(&gantryExtensionManualControl);
    wristSubsystem.setDefaultCommand(&wristControllerCommand);
    wristRollSubsystem.setDefaultCommand(&wristControllerCommand);
    cubeLift.setDefaultCommand(&cubeManualControl);

    suckSubsystem.setDefaultCommand(&suckOffCommand);
    releaseSubsystem.setDefaultCommand(&releaseOffCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startEngineerCommands(aruwsrc::engineer::Drivers *) {}

/* register io mappings here ------------------------------------------------*/
void registerEngineerIoMappings(aruwsrc::engineer::Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightUp);
    drivers->commandMapper.addMap(&rightDown);

    drivers->commandMapper.addMap(&oneCube);
    drivers->commandMapper.addMap(&twoCube);
    drivers->commandMapper.addMap(&threeCube);

    drivers->commandMapper.addMap(&wristFoldIn);
    drivers->commandMapper.addMap(&wristFoldOut);
}
}  // namespace control
}  // namespace aruwsrc

namespace aruwsrc::engineer
{
void initSubsystemCommands(aruwsrc::engineer::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &aruwsrc::control::remoteSafeDisconnectFunction);
    aruwsrc::control::initializeSubsystems();
    aruwsrc::control::registerEngineerSubsystems(drivers);
    aruwsrc::control::setDefaultEngineerCommands(drivers);
    aruwsrc::control::startEngineerCommands(drivers);
    aruwsrc::control::registerEngineerIoMappings(drivers);
}
}  // namespace aruwsrc::engineer

#endif
