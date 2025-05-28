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

#include "aruwsrc/communication/sensors/beam_break/beam_break.hpp"
#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/communication/sensors/voltage/fake_voltage_sensor.hpp"
#include "aruwsrc/control/bounded-subsystem/homing_command.hpp"
#include "aruwsrc/control/bounded-subsystem/trigger/limit_switch_trigger.hpp"
#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "aruwsrc/control/chassis/mecanum_chassis_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/engineer/arm/arm_controller_command.hpp"
#include "aruwsrc/robot/engineer/arm/arm_extension_subsystem.hpp"
#include "aruwsrc/robot/engineer/arm/arm_lift_subsystem.hpp"
#include "aruwsrc/robot/engineer/arm/joint_subsystem.hpp"
#include "aruwsrc/robot/engineer/arm/wrist_subsystem.hpp"
#include "aruwsrc/robot/engineer/cube_lift/cube_move_manual_command.hpp"
#include "aruwsrc/robot/engineer/cube_lift/cube_move_position_command.hpp"
#include "aruwsrc/robot/engineer/cube_lift/cube_storage_subsystem.hpp"
#include "aruwsrc/robot/engineer/cube_lift/engineer_lift_constants.hpp"
#include "aruwsrc/robot/engineer/engineer_drivers.hpp"
#include "aruwsrc/robot/engineer/engineer_gantry_constants.hpp"

using namespace tap::gpio;
using tap::communication::serial::Remote;
using tap::control::CommandMapper;
using namespace aruwsrc::engineer;
using namespace aruwsrc::robot::engineer;
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
tap::motor::DjiMotor storageLiftMotor(
    drivers(),
    CUBE_LIFT_MOTOR_ID,
    LIFT_MOTOR_CAN_BUS,
    true,
    "Lifting Motor",
    false,
    1 / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);
aruwsrc::communication::sensors::beam_break::DigitalBeamBreak cubeLiftLimit(
    &(drivers()->digital),
    CUBELIFT_LIMITSWITCH_PORT,
    true);
LimitSwitchTrigger cubeLiftTrigger(&cubeLiftLimit);
/* define subsystems --------------------------------------------------------*/
CubeStorageSubsystem cubeLift(drivers(), storageLiftMotor, cubeLiftTrigger, 0);

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

tap::communication::sensors::current::AnalogCurrentSensor currentSensor(
    {&drivers()->analog,
     aruwsrc::chassis::CURRENT_SENSOR_PIN,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_MV_PER_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_ZERO_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA});

aruwsrc::chassis::MecanumChassisSubsystem chassis(
    drivers(),
    &currentSensor,
    &voltageSensor,
    leftFrontChassisMotor,
    leftBackChassisMotor,
    rightFrontChassisMotor,
    rightBackChassisMotor,
    aruwsrc::chassis::WHEEL_VELOCITY_PID_CONFIG);

tap::motor::DjiMotor storageLiftMotor(
    drivers(),
    CUBE_LIFT_MOTOR_ID,
    LIFT_MOTOR_CAN_BUS,
    true,
    "Lifting Motor",
    false,
    1 / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

aruwsrc::communication::sensors::beam_break::DigitalBeamBreak cubeLiftLimit(
    &(drivers()->digital),
    CUBELIFT_LIMITSWITCH_PORT,
    true);
LimitSwitchTrigger cubeLiftTrigger(&cubeLiftLimit);
/* define subsystems --------------------------------------------------------*/
CubeStorageSubsystem cubeLift(
    drivers(),
    storageLiftMotor,
    LIFT_MOTOR_PID_CONFIG,
    LIFT_HOMING_PID_CONFIG,
    cubeLiftTrigger);

tap::motor::DjiMotor engineerWristRollMotor(
    drivers(),
    aruwsrc::engineer::WRIST_ROLL_MOTOR_ID,
    aruwsrc::engineer::CAN_BUS_GANTRY,
    false,
    "Wrist Roll Motor",
    false,
    1.0f / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor engineerWristLeftMotor(
    drivers(),
    aruwsrc::engineer::WRIST_LEFT_MOTOR_ID,
    aruwsrc::engineer::CAN_BUS_GANTRY,
    false,
    "Wrist Left Motor",
    false,
    1.0f / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor engineerWristRightMotor(
    drivers(),
    aruwsrc::engineer::WRIST_RIGHT_MOTOR_ID,
    aruwsrc::engineer::CAN_BUS_GANTRY,
    false,
    "Wrist Right Motor",
    false,
    1.0f / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::encoder::CanEncoder engineerWristPitchEncoder(
    drivers(),
    aruwsrc::engineer::WRIST_PITCH_ENCODER_ID,
    aruwsrc::chassis::CAN_BUS_MOTORS,
    false,
    1,
    WRIST_HOME_PITCH * 4096.0f / (M_PI * 2));

tap::encoder::CanEncoder engineerWristYawEncoder(
    drivers(),
    aruwsrc::engineer::WRIST_YAW_ENCODER_ID,
    aruwsrc::chassis::CAN_BUS_MOTORS,
    false,
    1,
    WRIST_HOME_YAW * 4096.0f / (M_PI * 2));

tap::motor::DjiMotor engineerGantryLiftLeftMotor(
    drivers(),
    aruwsrc::engineer::GANTRY_LIFT_LEFT_MOTOR_ID,
    aruwsrc::engineer::CAN_BUS_GANTRY,
    true,
    "Gantry Lift Left Motor",
    false,
    1.0f / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor engineerGantryLiftRightMotor(
    drivers(),
    aruwsrc::engineer::GANTRY_LIFT_RIGHT_MOTOR_ID,
    aruwsrc::engineer::CAN_BUS_GANTRY,
    false,
    "Gantry Lift Right Motor",
    false,
    1.0f / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

aruwsrc::communication::sensors::beam_break::DigitalBeamBreak liftLimitSwitch(
    &drivers()->digital,
    aruwsrc::engineer::GANTRY_LIFT_LIMIT_SWITCH_PIN);

LimitSwitchTrigger liftLimitSwitchTrigger(&liftLimitSwitch);

tap::motor::DjiMotor cubeStorageLiftMotor(
    drivers(),
    tap::motor::MotorId::MOTOR7,
    tap::can::CanBus::CAN_BUS2,
    false,
    "Cube Storage Lift Motor",
    false,
    1.0f / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor engineerGantryExtensionMotor(
    drivers(),
    aruwsrc::engineer::GANTRY_EXTENSION_MOTOR_ID,
    aruwsrc::engineer::CAN_BUS_GANTRY,
    true,
    "Gantry Extension Motor",
    false,
    1.0f / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

aruwsrc::communication::sensors::beam_break::DigitalBeamBreak extensionLimitSwitch(
    &drivers()->digital,
    aruwsrc::engineer::GANTRY_EXTENSION_LIMIT_SWITCH_PIN);

LimitSwitchTrigger extensionLimitSwitchTrigger(&extensionLimitSwitch);

WristSubsystem wristSubsystem(
    drivers(),
    engineerWristLeftMotor,
    engineerWristRightMotor,
    engineerWristPitchEncoder,
    engineerWristYawEncoder,
    aruwsrc::engineer::WRIST_PITCH_CONFIG,
    aruwsrc::engineer::WRIST_YAW_CONFIG,
    aruwsrc::engineer::WRIST_MIN_PITCH,
    aruwsrc::engineer::WRIST_MAX_PITCH,
    aruwsrc::engineer::WRIST_MIN_YAW,
    aruwsrc::engineer::WRIST_MAX_YAW,
    aruwsrc::engineer::WRIST_RATIO);

ArmLiftSubsystem armLiftSubsystem(
    drivers(),
    engineerGantryLiftLeftMotor,
    engineerGantryLiftRightMotor,
    aruwsrc::engineer::GANTRY_LIFT_POS_CONFIG,
    aruwsrc::engineer::GANTRY_LIFT_BALANCE_CONFIG,
    liftLimitSwitchTrigger,
    1.0f,
    0.0f,
    GANTRY_LIFT_MAX_SETPOINT);

ArmExtensionSubsystem armExtensionSubsystem(
    drivers(),
    engineerGantryExtensionMotor,
    aruwsrc::engineer::GANTRY_EXTENSION_CONFIG,
    extensionLimitSwitchTrigger,
    1.0f,  // todo
    0.0f,
    GANTRY_EXTENSION_MAX_SETPOINT);

JointSubsystem wristRollSubsystem(
    drivers(),
    engineerWristRollMotor,
    aruwsrc::engineer::WRIST_ROLL_CONFIG);

/* define commands ----------------------------------------------------------*/
HomingCommand cubeLiftHome(cubeLift);
HomingCommand gantryLiftHome(armLiftSubsystem);
HomingCommand gantryExtensionHome(armExtensionSubsystem);

CubeMoveManualCommand cubeManualControl(
    cubeLift,
    &drivers()->controlOperatorInterface,
    MANUAL_MOVE_SPEED);
CubeMovePositionCommand oneCubePosition(cubeLift, ONE_CUBE_SETPOINT);
CubeMovePositionCommand twoCubePosition(cubeLift, TWO_CUBE_SETPOINT);
CubeMovePositionCommand threeCubePosition(cubeLift, THREE_CUBE_SETPOINT);

aruwsrc::chassis::ChassisDriveCommand chassisDriveCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis);

control::engineer::ArmControllerCommand armControllerCommand(
    armLiftSubsystem,
    armExtensionSubsystem,
    wristRollSubsystem,
    wristSubsystem,
    &drivers()->controlOperatorInterface,
    aruwsrc::engineer::GANTRY_LIFT_SCALING_FACTOR,
    aruwsrc::engineer::GANTRY_EXTENSION_SCALING_FACTOR,
    aruwsrc::engineer::WRIST_ROLL_SCALING_FACTOR,
    aruwsrc::engineer::WRIST_PITCH_SCALING_FACTOR,
    aruwsrc::engineer::WRIST_YAW_SCALING_FACTOR);

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

tap::control::HoldCommandMapping rightUp(
    drivers(),
    {&cubeLiftHome, &gantryLiftHome, &gantryExtensionHome},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

tap::control::HoldCommandMapping leftMidRightMid(
    drivers(),
    {&cubeManualControl},
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::MID));

// tap::control::HoldCommandMapping leftDownRightUp(
//     drivers(),
//     {&oneCubePosition},
//     RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::UP));

// tap::control::HoldCommandMapping leftDownRightMid(
//     drivers(),
//     {&twoCubePosition},
//     RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::MID));

// tap::control::HoldCommandMapping leftDownRightDown(
//     drivers(),
//     {&threeCubePosition},
//     RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::DOWN));

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    chassis.initialize();
    armLiftSubsystem.initialize();
    armExtensionSubsystem.initialize();
    wristRollSubsystem.initialize();
    wristSubsystem.initialize();
    cubeLift.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerEngineerSubsystems(aruwsrc::engineer::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&armLiftSubsystem);
    drivers->commandScheduler.registerSubsystem(&armExtensionSubsystem);
    drivers->commandScheduler.registerSubsystem(&wristRollSubsystem);
    drivers->commandScheduler.registerSubsystem(&wristSubsystem);
    drivers->commandScheduler.registerSubsystem(&cubeLift);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultEngineerCommands(aruwsrc::engineer::Drivers *)
{
    chassis.setDefaultCommand(&chassisDriveCommand);
    armLiftSubsystem.setDefaultCommand(&armControllerCommand);
    armExtensionSubsystem.setDefaultCommand(&armControllerCommand);
    wristSubsystem.setDefaultCommand(&armControllerCommand);
    wristRollSubsystem.setDefaultCommand(&armControllerCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startEngineerCommands(aruwsrc::engineer::Drivers *)
{
    drivers()->commandScheduler.addCommand(&armControllerCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerEngineerIoMappings(aruwsrc::engineer::Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightUp);
    drivers->commandMapper.addMap(&leftMidRightMid);
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
