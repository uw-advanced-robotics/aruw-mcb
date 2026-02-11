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

// Guys please dont make fun of me
#if defined(TARGET_ENGINEER)

#include "aruwsrc/util_macros.hpp"

#if defined(TARGET_ENGINEER)

#include "tap/communication/gpio/digital.hpp"
#include "tap/communication/sensors/encoder/can_encoder/can_encoder.hpp"
#include "tap/communication/sensors/limit_switch/limit_switch_interface.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/command_scheduler.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/sequential_command.hpp"

#include "aruwsrc/communication/sensors/beam_break/beam_break.hpp"
#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/communication/sensors/voltage/fake_voltage_sensor.hpp"
#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "aruwsrc/control/client-display/client_display_command.hpp"
#include "aruwsrc/control/client-display/client_display_subsystem.hpp"
#include "aruwsrc/control/cycle_state_command_mapping.hpp"
#include "aruwsrc/control/digital/digital_out_command.hpp"
#include "aruwsrc/control/digital/digital_out_subsystem.hpp"
#include "aruwsrc/control/digital/digital_out_toggle_command.hpp"
#include "aruwsrc/control/digital/dual_digital_out_subsystem.hpp"
#include "aruwsrc/control/joint/homing/homing_command.hpp"
#include "aruwsrc/control/joint/homing/trigger/limit_switch_trigger.hpp"
#include "aruwsrc/control/joint/homing/trigger_homed_dual_joint_subsystem.hpp"
#include "aruwsrc/control/joint/joint_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/engineer/cube_storage/cube_position_digital_out_command.hpp"
#include "aruwsrc/robot/engineer/cube_storage/cube_storage_subsystem.hpp"
#include "aruwsrc/robot/engineer/cube_storage/engineer_cube_storage_constants.hpp"
#include "aruwsrc/robot/engineer/cube_storage/select_cube_position_command.hpp"
#include "aruwsrc/robot/engineer/engineer_drivers.hpp"
#include "aruwsrc/robot/engineer/engineer_extension_constants.hpp"
#include "aruwsrc/robot/engineer/engineer_setpoint_constants.hpp"
#include "aruwsrc/robot/engineer/engineer_wrist_constants.hpp"
#include "aruwsrc/robot/engineer/score_position_command.hpp"
#include "aruwsrc/robot/engineer/setpoint_move_manual_command.hpp"
#include "aruwsrc/robot/engineer/setpoint_move_position_command.hpp"
#include "aruwsrc/robot/engineer/turret/engineer_turret_subsystem.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_controller_command.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_move_position_command.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_setpoints_command.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_subsystem.hpp"

// #include "aruwsrc/robot/engineer/turret/constants/engineer_turret_constants.hpp"
#include "aruwsrc/algorithms/odometry/otto_chassis_world_yaw_observer.hpp"
#include "aruwsrc/control/chassis/chassis_autorotate_command.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"

// check which of these r important
#include "aruwsrc/control/buzzer/buzzer_subsystem.hpp"
#include "aruwsrc/control/buzzer/note_sequence_command.hpp"
#include "aruwsrc/control/buzzer/note_sequences.hpp"
#include "aruwsrc/control/chassis/x_drive_chassis_subsystem.hpp"
#include "aruwsrc/control/governor/imu_calibrate_done_governor.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/user/turret_quick_turn_command.hpp"
#include "aruwsrc/control/turret/user/turret_user_world_relative_command.hpp"

using namespace aruwsrc::control::client_display;
using namespace aruwsrc::control::client_display::indicators;
using namespace aruwsrc::control::digital;
using namespace aruwsrc::control::joint;
using namespace aruwsrc::control::joint::homing;
using namespace aruwsrc::control::joint::homing::trigger;
using namespace aruwsrc::engineer;
using namespace aruwsrc::engineer::wrist;
using namespace aruwsrc::engineer::cube_storage;
using namespace tap::control;
using namespace tap::gpio;

using tap::communication::serial::Remote;
using tap::control::CommandMapper;

using namespace aruwsrc::control::turret;
using namespace aruwsrc::algorithms::odometry;
using namespace aruwsrc::control::buzzer;
using namespace aruwsrc::control::chassis;

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
inline aruwsrc::communication::can::TurretMCBCanComm &getTurretMCBCanComm()
{
    return drivers()->turretMCBCanCommBus1;
}

tap::motor::DjiMotor pitchTurretMotor(
    drivers(),
    PITCH_MOTOR_ID,
    CAN_BUS_MOTORS,
    true,
    "Pitch Turret",
    true,
    1,
    PITCH_MOTOR_CONFIG.startEncoderValue);

tap::motor::DjiMotor yawTurretMotor(
    drivers(),
    YAW_MOTOR_ID,
    CAN_BUS_MOTORS,
    false,
    "Yaw Turret",
    true,
    1,
    YAW_MOTOR_CONFIG.startEncoderValue);

EngineerTurretSubsystem engTurret(
    drivers(),
    &pitchTurretMotor,
    &yawTurretMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG,
    &getTurretMCBCanComm());

aruwsrc::algorithms::odometry::OttoChassisWorldYawObserver yawObserver(engTurret);

aruwsrc::communication::sensors::voltage::FakeVoltageSensor voltageSensor;

tap::motor::DjiMotor leftFrontChassisMotor(
    drivers(),
    aruwsrc::control::chassis::LEFT_FRONT_MOTOR_ID,
    aruwsrc::control::chassis::CAN_BUS_MOTORS,
    false,
    "Left Front Chassis Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor leftBackChassisMotor(
    drivers(),
    aruwsrc::control::chassis::LEFT_BACK_MOTOR_ID,
    aruwsrc::control::chassis::CAN_BUS_MOTORS,
    false,
    "Left Back Chassis Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor rightFrontChassisMotor(
    drivers(),
    aruwsrc::control::chassis::RIGHT_FRONT_MOTOR_ID,
    aruwsrc::control::chassis::CAN_BUS_MOTORS,
    false,
    "Right Front Chassis Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor rightBackChassisMotor(
    drivers(),
    aruwsrc::control::chassis::RIGHT_BACK_MOTOR_ID,
    aruwsrc::control::chassis::CAN_BUS_MOTORS,
    false,
    "Right Back Chassis Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::communication::sensors::current::AnalogCurrentSensor currentSensor(
    {&drivers()->analog,
     aruwsrc::control::chassis::CURRENT_SENSOR_PIN,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_MV_PER_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_ZERO_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA});

tap::motor::DjiMotor cubeStorageMotor(
    drivers(),
    CUBE_STORAGE_MOTOR_ID,
    CUBE_STORAGE_MOTOR_CAN_BUS,
    true,
    "Cube Storage Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

aruwsrc::communication::sensors::beam_break::DigitalBeamBreak cubeStorageLimit(
    &(drivers()->digital),
    CUBE_STORAGE_LIMITSWITCH_PORT,
    true);

LimitSwitchTrigger cubeStorageTrigger(&cubeStorageLimit);

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
    aruwsrc::control::chassis::CAN_BUS_MOTORS,
    false,
    1,
    WRIST_HOME_PITCH);

tap::encoder::CanEncoder wristYawEncoder(
    drivers(),
    aruwsrc::engineer::WRIST_YAW_ENCODER_ID,
    aruwsrc::control::chassis::CAN_BUS_MOTORS,
    false,
    1,
    WRIST_HOME_YAW);

tap::motor::DjiMotor extensionMotor(
    drivers(),
    aruwsrc::engineer::EXTENSION_MOTOR_ID,
    aruwsrc::engineer::CAN_BUS_EXTENSION,
    true,  // inverted? test
    "Extension Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

aruwsrc::communication::sensors::beam_break::DigitalBeamBreak extensionLimit(
    &drivers()->digital,
    aruwsrc::engineer::EXTENSION_LIMIT_SWITCH_PIN,
    true);

LimitSwitchTrigger extensionTrigger(&extensionLimit);

/* define subsystems --------------------------------------------------------*/

aruwsrc::control::chassis::XDriveChassisSubsystem xDriveChassis(
    drivers(),
    &currentSensor,
    &voltageSensor,
    leftFrontChassisMotor,
    leftBackChassisMotor,
    rightFrontChassisMotor,
    rightBackChassisMotor,
    aruwsrc::control::chassis::WHEEL_VELOCITY_PID_CONFIG,
    WHEEL_RADIUS,
    WHEELBASE_RADIUS);

// this could be useful i think

aruwsrc::control::chassis::ChassisAutorotateCommand chassisAutorotateCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &xDriveChassis,
    &engTurret.yawMotor,
    aruwsrc::control::chassis::ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180);

aruwsrc::control::turret::algorithms::ChassisFrameTurretController<
    aruwsrc::control::turret::algorithms::Axis::PITCH>
    chassisFramePitchTurretController(engTurret.pitchMotor, chassis_rel::PITCH_PID_CONFIG);

aruwsrc::control::turret::algorithms::ChassisFrameTurretController<
    aruwsrc::control::turret::algorithms::Axis::YAW>
    chassisFrameYawTurretController(engTurret.yawMotor, chassis_rel::YAW_PID_CONFIG);

BuzzerSubsystem engineerBuzzer(drivers());

NoteSequenceCommand imuCalibrateSuccessBuzzCommand(
    engineerBuzzer,
    IMU_CALIBRATE_SUCCESS_NOTES,
    IMU_CALIBRATE_SUCCESS_NOTE_LENGTH_MS);

NoteSequenceCommand imuCalibrateFailBuzzCommand(
    engineerBuzzer,
    IMU_CALIBRATE_FAIL_NOTES,
    IMU_CALIBRATE_FAIL_NOTE_LENGTH_MS);

imu::ImuCalibrateCommand imuCalibrateCommand(
    drivers(),
    {{
        &getTurretMCBCanComm(),
        &engTurret,
        &chassisFrameYawTurretController,
        &chassisFramePitchTurretController,
        true,
    }},
    &xDriveChassis,
    imu::ImuCalibrateCommand::DEFAULT_VELOCITY_ZERO_THRESHOLD,
    imu::ImuCalibrateCommand::DEFAULT_POSITION_ZERO_THRESHOLD,
    &imuCalibrateSuccessBuzzCommand,
    &imuCalibrateFailBuzzCommand,
    nullptr,
    // {&drivers()->ism330});
    {&drivers()->mpu6500});

aruwsrc::control::governor::IMUCalibrateDoneGovernor imuCalibrateDoneGovernor(
    drivers(),
    imuCalibrateCommand);

CubeStorageSubsystem cubeStorage(
    drivers(),
    cubeStorageMotor,
    cubeStorageTrigger,
    CUBE_STORAGE_CONFIG);

WristSubsystem wristSubsystem(
    drivers(),
    wristLeftMotor,
    wristRightMotor,
    wristPitchEncoder,
    wristYawEncoder,
    WRIST_CONFIG);

TriggerHomedJointSubsystem extensionSubsystem(
    drivers(),
    extensionMotor,
    extensionTrigger,
    EXTENSION_CONFIG);

JointSubsystem wristRollSubsystem(drivers(), wristRollMotor, WRIST_ROLL_CONFIG);

// update vals
DualDigitalOutSubsystem leftSuckSubsystem(
    drivers(),
    drivers()->digital,
    tap::gpio::Digital::OutputPin::Y,
    true,
    tap::gpio::Digital::OutputPin::Z,
    false);
DualDigitalOutSubsystem rightSuckSubsystem(
    drivers(),
    drivers()->digital,
    tap::gpio::Digital::OutputPin::Y,
    true,
    tap::gpio::Digital::OutputPin::Z,
    false);

/* define client display / HUD related items --------------------------------*/
ClientDisplaySubsystem clientDisplay(drivers());
tap::communication::serial::RefSerialTransmitter refSerialTransmitter(drivers());

/* define commands ----------------------------------------------------------*/
HomingCommand cubeStorageHome(cubeStorage);
HomingCommand extensionHome(extensionSubsystem);

SetpointMoveManualCommand cubeManualControl(
    cubeStorage,
    &drivers()->controlOperatorInterface,
    CUBE_STORAGE_MOVE_SPEED,
    SetpointType::CUBE_STORAGE);

SetpointMoveManualCommand extensionManualControl(
    extensionSubsystem,
    &drivers()->controlOperatorInterface,
    EXTENSION_MOVE_SPEED,
    SetpointType::EXTENSION);

aruwsrc::control::chassis::ChassisDriveCommand chassisDriveCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &xDriveChassis);

WristControllerCommand wristControllerCommand(
    wristRollSubsystem,
    wristSubsystem,
    &drivers()->controlOperatorInterface,
    WRIST_ROLL_SCALING_FACTOR,
    WRIST_PITCH_SCALING_FACTOR,
    WRIST_YAW_SCALING_FACTOR);

// wrist fold in commands are not fully tuned yet
WristSetpointsCommand wristFoldInCommand(
    wristSubsystem,
    {WRIST_BOTTOM_SETPOINT, WRIST_TOP_SETPOINT, WRIST_IN_SETPOINT});

WristSetpointsCommand wristFoldOutCommand(
    wristSubsystem,
    {WRIST_TOP_SETPOINT, WRIST_BOTTOM_SETPOINT, WRIST_OUT_SETPOINT});

// commands here for sequences, but setpoints never tuned
SetpointMovePositionCommand extensionInCommand(extensionSubsystem, 2);
SetpointMovePositionCommand extensionOutCommand(extensionSubsystem, 2);

// commands for pickup/scoring positions
SetpointMovePositionCommand extensionOut(extensionSubsystem, EXTENSION_SCORE);
SetpointMovePositionCommand extensionIn(extensionSubsystem, EXTENSION_PICKUP);

// rotating cube storage
SetpointMovePositionCommand leftCubePosition(cubeStorage, CUBE_STORAGE_LEFT_SETPOINT);
SetpointMovePositionCommand rightCubePosition(cubeStorage, CUBE_STORAGE_RIGHT_SETPOINT);
SetpointMovePositionCommand centerCubePosition(cubeStorage, CUBE_STORAGE_CENTER_SETPOINT);

WristMovePositionCommand wristDown(
    wristSubsystem,
    WRIST_PITCH_PICKUP,
    WRIST_YAW_PICKUP);  // tuned to align for better suction
WristMovePositionCommand wristOut(wristSubsystem, WRIST_PITCH_SCORE, WRIST_YAW_SCORE);

ScorePositionCommand scorePositionCommand(extensionSubsystem, wristSubsystem, wristRollSubsystem);

SelectCubePositionCommand selectCubeAddPositionCommand(cubeStorage, wristRollSubsystem, true);
SelectCubePositionCommand selectCubeRemovePositionCommand(cubeStorage, wristRollSubsystem, false);
CubePositionDigitalOutCommand cubeStorageSuckOnCommand(
    cubeStorage,
    leftSuckSubsystem,
    rightSuckSubsystem,
    true);
CubePositionDigitalOutCommand cubeStorageSuckOffCommand(
    cubeStorage,
    leftSuckSubsystem,
    rightSuckSubsystem,
    false);

SequentialCommand<4> storeCubeCommand(std::array<Command *, 4>{{
    &selectCubeAddPositionCommand,
    &cubeStorageSuckOnCommand,
    // hand down
    // hand release cube
    // hand up
    &centerCubePosition,
}});

SequentialCommand<4> removeCubeCommand(std::array<Command *, 4>{{
    &selectCubeRemovePositionCommand,
    // hand suction on
    // hand down
    &cubeStorageSuckOffCommand,
    // hand up
    &centerCubePosition,
}});

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

tap::control::PressCommandMapping leftUp(
    drivers(),
    {&cubeStorageHome, &extensionHome},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

tap::control::PressCommandMapping vPressed(
    drivers(),
    {&extensionOut, &wristDown},
    RemoteMapState({Remote::Key::V}));

tap::control::PressCommandMapping bPressed(
    drivers(),
    {&extensionIn, &wristOut},
    RemoteMapState({Remote::Key::B}));

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    xDriveChassis.initialize();
    extensionSubsystem.initialize();
    wristRollSubsystem.initialize();
    wristSubsystem.initialize();
    cubeStorage.initialize();
    leftSuckSubsystem.initialize();
    rightSuckSubsystem.initialize();
    // clientDicsplay.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerEngineerSubsystems(aruwsrc::engineer::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&xDriveChassis);
    drivers->commandScheduler.registerSubsystem(&extensionSubsystem);
    drivers->commandScheduler.registerSubsystem(&wristRollSubsystem);
    drivers->commandScheduler.registerSubsystem(&wristSubsystem);
    drivers->commandScheduler.registerSubsystem(&cubeStorage);
    drivers->commandScheduler.registerSubsystem(&leftSuckSubsystem);
    drivers->commandScheduler.registerSubsystem(&rightSuckSubsystem);
    // drivers->commandScheduler.registerSubsystem(&clientDisplay);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultEngineerCommands(aruwsrc::engineer::Drivers *)
{
    xDriveChassis.setDefaultCommand(&chassisDriveCommand);
    extensionSubsystem.setDefaultCommand(&extensionManualControl);
    wristSubsystem.setDefaultCommand(&wristControllerCommand);
    wristRollSubsystem.setDefaultCommand(&wristControllerCommand);
    cubeStorage.setDefaultCommand(&cubeManualControl);

    // clientDisplay.setDefaultCommand(&clientDisplayCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startEngineerCommands(aruwsrc::engineer::Drivers *) {}

/* register io mappings here ------------------------------------------------*/
void registerEngineerIoMappings(aruwsrc::engineer::Drivers *drivers)
{
    // drivers->commandMapper.addMap(&storeCube);
    // drivers->commandMapper.addMap(&retrieveCube);
    // drivers->commandMapper.addMap(&cyclePositions);
    // drivers->commandMapper.addMap(&cPressed);
    drivers->commandMapper.addMap(&leftUp);
    drivers->commandMapper.addMap(&vPressed);
    drivers->commandMapper.addMap(&bPressed);
    // drivers->commandMapper.addMap(&wristFoldIn);
    // drivers->commandMapper.addMap(&wristFoldOut);
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
#endif