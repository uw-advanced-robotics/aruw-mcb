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
#include <memory>

#include "tap/communication/gpio/digital.hpp"
#include "tap/communication/sensors/encoder/can_encoder/can_encoder.hpp"
#include "tap/communication/sensors/limit_switch/limit_switch_interface.hpp"
#include "tap/control/command_composition_helper.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/command_scheduler.hpp"
#include "tap/control/instant_command.hpp"
#include "tap/control/remote_map_state.hpp"
#include "tap/control/sequential_command.hpp"
#include "tap/control/trigger.hpp"
#include "tap/control/trigger_helpers.hpp"

#include "aruwsrc/algorithms/odometry/otto_chassis_world_yaw_observer.hpp"
#include "aruwsrc/algorithms/odometry/three_deadwheel_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/communication/mcb-lite/motor/virtual_dji_motor.hpp"
#include "aruwsrc/communication/mcb-lite/motor/virtual_servo.hpp"
#include "aruwsrc/communication/mcb-lite/virtual_analog_sensor.hpp"
#include "aruwsrc/communication/mcb-lite/virtual_digital_limit_switch.hpp"
#include "aruwsrc/communication/sensors/beam_break/beam_break.hpp"
#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/communication/sensors/encoder/lamprey_encoder.hpp"
#include "aruwsrc/communication/sensors/voltage/fake_voltage_sensor.hpp"
#include "aruwsrc/control/autotune/lamprey_autotune.hpp"
#include "aruwsrc/control/buzzer/buzzer_subsystem.hpp"
#include "aruwsrc/control/buzzer/note_sequence_command.hpp"
#include "aruwsrc/control/buzzer/note_sequences.hpp"
#include "aruwsrc/control/chassis/auto_nav_command.hpp"
#include "aruwsrc/control/chassis/chassis_autorotate_command.hpp"
#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "aruwsrc/control/chassis/x_drive_chassis_subsystem.hpp"
#include "aruwsrc/control/client-display/client_display_command.hpp"
#include "aruwsrc/control/client-display/client_display_subsystem.hpp"
#include "aruwsrc/control/cycle_state_command_mapping.hpp"
#include "aruwsrc/control/digital/digital_out_command.hpp"
#include "aruwsrc/control/digital/digital_out_subsystem.hpp"
#include "aruwsrc/control/digital/digital_out_toggle_command.hpp"
#include "aruwsrc/control/digital/dual_digital_out_subsystem.hpp"
#include "aruwsrc/control/governor/imu_calibrate_done_governor.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "aruwsrc/control/joint/homing/homing_command.hpp"
#include "aruwsrc/control/joint/homing/trigger/limit_switch_trigger.hpp"
#include "aruwsrc/control/joint/homing/trigger_homed_dual_joint_subsystem.hpp"
#include "aruwsrc/control/joint/joint_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/turret_motor.hpp"
#include "aruwsrc/control/turret/user/turret_quick_turn_command.hpp"
#include "aruwsrc/control/turret/user/turret_user_world_relative_command.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/engineer/algorithms/engineer_transform_subsystem.hpp"
#include "aruwsrc/robot/engineer/algorithms/engineer_transforms.hpp"
#include "aruwsrc/robot/engineer/algorithms/inverse_kinematics/manual_ik_command.hpp"
#include "aruwsrc/robot/engineer/algorithms/inverse_kinematics/trajectory_ik_command.hpp"
#include "aruwsrc/robot/engineer/binned_alignment_command.hpp"
#include "aruwsrc/robot/engineer/cube_storage/cube_position_digital_out_command.hpp"
#include "aruwsrc/robot/engineer/cube_storage/cube_storage_subsystem.hpp"
#include "aruwsrc/robot/engineer/cube_storage/engineer_cube_storage_constants.hpp"
#include "aruwsrc/robot/engineer/cube_storage/select_cube_position_command.hpp"
#include "aruwsrc/robot/engineer/engineer_drivers.hpp"
#include "aruwsrc/robot/engineer/engineer_extension_constants.hpp"
#include "aruwsrc/robot/engineer/engineer_setpoint_constants.hpp"
#include "aruwsrc/robot/engineer/engineer_turret_constants.hpp"
#include "aruwsrc/robot/engineer/engineer_turret_subsystem.hpp"
#include "aruwsrc/robot/engineer/engineer_wrist_constants.hpp"
#include "aruwsrc/robot/engineer/score_position_command.hpp"
#include "aruwsrc/robot/engineer/setpoint_move_manual_command.hpp"
#include "aruwsrc/robot/engineer/setpoint_move_position_command.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_controller_command.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_move_position_command.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_setpoints_command.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_subsystem.hpp"
#include "aruwsrc/util_macros.hpp"

using namespace aruwsrc::algorithms::odometry;
using namespace aruwsrc::control::turret::algorithms;
using namespace aruwsrc::control::buzzer;
using namespace aruwsrc::control::chassis;
using namespace aruwsrc::control::client_display;
using namespace aruwsrc::control::client_display::indicators;
using namespace aruwsrc::control::digital;
using namespace aruwsrc::control::joint;
using namespace aruwsrc::control::joint::homing;
using namespace aruwsrc::control::joint::homing::trigger;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::engineer;
using namespace aruwsrc::engineer::algorithms;
using namespace aruwsrc::engineer::cube_storage;
using namespace aruwsrc::engineer::wrist;
using namespace tap::control;
using namespace tap::gpio;

using tap::algorithms::transforms::Transform;
using tap::communication::serial::Remote;
using tap::control::CommandMapper;

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
aruwsrc::communication::mcb_lite::VirtualCanEncoder turretPitchEncoder(
    drivers(),
    tap::encoder::CanEncoderId::ID2,
    &drivers()->mcbLite,
    tap::can::CanBus::CAN_BUS2,
    false,
    1.0f,
    PITCH_TURRET_ENCODER_HOME);

tap::motor::DjiMotor pitchTurretMotor(
    drivers(),
    PITCH_MOTOR_ID,
    tap::can::CanBus::CAN_BUS1,
    false,
    "Pitch Turret",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508* PITCH_TURRET_GEAR_RATIO,
    0,
    &turretPitchEncoder);

tap::motor::DjiMotor yawTurretMotor(
    drivers(),
    YAW_MOTOR_ID,
    CAN_BUS_MOTORS,
    false,
    "Yaw Turret",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508* YAW_TURRET_GEAR_RATIO,
    YAW_MOTOR_CONFIG.startEncoderValue);

tap::motor::DjiMotor extensionMotor(
    drivers(),
    aruwsrc::engineer::EXTENSION_MOTOR_ID,
    aruwsrc::engineer::CAN_BUS_EXTENSION,
    true,
    "Extension Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

aruwsrc::communication::sensors::beam_break::DigitalBeamBreak extensionLimit(
    &drivers()->digital,
    aruwsrc::engineer::EXTENSION_LIMIT_SWITCH_PIN,
    true);

LimitSwitchTrigger extensionTrigger(&extensionLimit);

TriggerHomedJointSubsystem extensionSubsystem(
    drivers(),
    extensionMotor,
    extensionTrigger,
    EXTENSION_CONFIG);

float getLivePitchMinLimit();
float getLivePitchMaxLimit();

aruwsrc::control::turret::TurretMotor pitchEngTurretMotor(
    &pitchTurretMotor,
    PITCH_MOTOR_CONFIG,
    getLivePitchMinLimit,
    getLivePitchMaxLimit);

aruwsrc::control::turret::TurretMotor yawEngTurretMotor(&yawTurretMotor, YAW_MOTOR_CONFIG);

EngineerTurretSubsystem engTurret(
    drivers(),
    pitchEngTurretMotor,
    yawEngTurretMotor,
    &drivers()->mcbLite.imu);

float getLivePitchMinLimit() { return getPitchMinLimit(extensionSubsystem.getPosition()); }

float getLivePitchMaxLimit() { return getPitchMaxLimit(extensionSubsystem.getPosition()); }

aruwsrc::algorithms::odometry::OttoChassisWorldYawObserver yawObserver(engTurret);

aruwsrc::communication::sensors::voltage::FakeVoltageSensor voltageSensor;

aruwsrc::communication::mcb_lite::VirtualAnalogSensor analogSensor(
    drivers(),
    tap::can::CanBus::CAN_BUS2,
    0x1D6);

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

tap::encoder::CanEncoder parallelOmniOne(
    drivers(),
    tap::encoder::CanEncoderId::ID0,
    tap::can::CanBus::CAN_BUS2,
    false);

tap::encoder::CanEncoder parallelOmniTwo(
    drivers(),
    tap::encoder::CanEncoderId::ID1,
    tap::can::CanBus::CAN_BUS2,
    true);

tap::encoder::CanEncoder perpendicularOmni(
    drivers(),
    tap::encoder::CanEncoderId::ID2,
    tap::can::CanBus::CAN_BUS2,
    false);
tap::encoder::CanEncoder pulleyEncoder(
    drivers(),
    tap::encoder::CanEncoderId::ID6,
    tap::can::CanBus::CAN_BUS2,
    true);
aruwsrc::communication::sensors::encoder::LampreyEncoder lampreyEncoder(
    drivers(),
    tap::encoder::CanEncoderId::ID7,
    tap::can::CanBus::CAN_BUS2,
    aruwsrc::control::turret::chassis_rel::LAMPREY_CALIBRATION_MAP,
    false);

tap::communication::sensors::current::AnalogCurrentSensor currentSensor(
    {&drivers()->analog,
     aruwsrc::control::chassis::CURRENT_SENSOR_PIN,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_MV_PER_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_ZERO_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA});

aruwsrc::communication::mcb_lite::VirtualDigitalLimitSwitch cubeStorageLimitSwitch(
    drivers()->mcbLite.digital,
    tap::gpio::Digital::InputPin::B,
    true);

LimitSwitchTrigger cubeStorageTrigger(&cubeStorageLimitSwitch);

aruwsrc::communication::mcb_lite::VirtualDigitalLimitSwitch extensionLimitSwitch(
    drivers()->mcbLite.digital,
    tap::gpio::Digital::InputPin::C,
    true);

aruwsrc::communication::mcb_lite::motor::VirtualDjiMotor cubeStorageMotor(
    drivers(),
    CUBE_STORAGE_MOTOR_ID,
    CUBE_STORAGE_MOTOR_CAN_BUS,
    &drivers()->mcbLite,
    true,
    "Cube Storage Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

// furthest wrist motor from end effector
tap::motor::DjiMotor wristMotorOne(
    drivers(),
    aruwsrc::engineer::WRIST_MOTOR_1_ID,
    aruwsrc::engineer::CAN_BUS_WRIST,
    false,
    "Wrist Motor 1",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M2006* WRIST_MOTOR_1_GEAR_RATIO);

// middle wrist motor
tap::motor::DjiMotor wristMotorTwo(
    drivers(),
    aruwsrc::engineer::WRIST_MOTOR_2_ID,
    aruwsrc::engineer::CAN_BUS_WRIST,
    true,
    "Wrist Motor 2",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M2006* WRIST_MOTOR_2_GEAR_RATIO);

// closest wrist motor to end effector
tap::motor::DjiMotor wristMotorThree(
    drivers(),
    aruwsrc::engineer::WRIST_THETA3_MOTOR_ID,
    aruwsrc::engineer::CAN_BUS_WRIST,
    false,
    "Wrist Theta3 Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M2006* WRIST_MOTOR_3_GEAR_RATIO);

aruwsrc::communication::mcb_lite::VirtualCanEncoder wristEncoderTheta1(
    drivers(),
    aruwsrc::engineer::WRIST_THETA1_ENCODER_ID,
    &drivers()->mcbLite,
    aruwsrc::engineer::CAN_BUS_WRIST,
    false,
    1,
    WRIST_HOME_THETA1);

tap::encoder::CanEncoder wristEncoderTheta2(
    drivers(),
    aruwsrc::engineer::WRIST_THETA2_ENCODER_ID,
    aruwsrc::engineer::CAN_BUS_WRIST,
    false,
    1,
    WRIST_HOME_THETA2);

/* define subsystems --------------------------------------------------------*/

aruwsrc::control::chassis::XDriveChassisSubsystem chassisSubsystem(
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

BuzzerSubsystem engineerBuzzer(drivers());

CubeStorageSubsystem cubeStorage(
    drivers(),
    cubeStorageMotor,
    cubeStorageTrigger,
    CUBE_STORAGE_CONFIG);

WristSubsystem wristSubsystem(
    drivers(),
    wristMotorOne,
    wristMotorTwo,
    wristMotorThree,
    wristEncoderTheta2,
    WRIST_CONFIG);

// update vals
DualDigitalOutSubsystem leftSuckSubsystem(
    drivers(),
    drivers()->digital,
    tap::gpio::Digital::OutputPin::Y,
    true,
    tap::gpio::Digital::OutputPin::Z,
    true);

DualDigitalOutSubsystem rightSuckSubsystem(
    drivers(),
    drivers()->digital,
    tap::gpio::Digital::OutputPin::Y,
    true,
    tap::gpio::Digital::OutputPin::Z,
    true);

aruwsrc::algorithms::odometry::ThreeDeadwheelOdometryObserver deadwheels(
    &parallelOmniOne,
    &parallelOmniTwo,
    &perpendicularOmni,
    DEADWHEEL_RADIUS);

aruwsrc::algorithms::odometry::ThreeDeadwheelKFOdometry2DSubsystem odometrySubsystem(
    *drivers(),
    deadwheels,
    yawObserver,
    drivers()->chassisIsm,
    INITIAL_CHASSIS_POSITION_X,
    INITIAL_CHASSIS_POSITION_Y,
    INITIAL_CHASSIS_ORIENTATION,
    parallelOneCenterToWheelDistance,
    parallelTwoCenterToWheelDistance,
    perpendicularCenterToWheelDistance,
    odomFrameToRobotFrame);

// transforms
EngineerTransforms transformer(
    odometrySubsystem,
    drivers()->chassisIsm,
    engTurret,
    drivers()->mcbLite.imu,
    extensionSubsystem,
    wristSubsystem,
    cubeStorage);

EngineerTransformSubsystem transformSubsystem(*drivers(), transformer);

ChassisAutoNavController autoNavController(
    *drivers(),
    chassisSubsystem,
    transformer.getWorldToChassis(),
    BEYBLADE_CONFIG,
    nullptr,
    0,
    0);

aruwsrc::control::chassis::ChassisAutorotateCommand chassisAutorotateCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassisSubsystem,
    &engTurret.yawMotor,
    aruwsrc::control::chassis::ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180);

ChassisFrameTurretController<tap::algorithms::transforms::Axis::PITCH>
    chassisFramePitchTurretController(engTurret.pitchMotor, chassis_rel::PITCH_PID_CONFIG);

ChassisFrameTurretController<tap::algorithms::transforms::Axis::YAW>
    chassisFrameYawTurretController(engTurret.yawMotor, chassis_rel::YAW_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretImuPosPid(
    world_rel_turret_imu::PITCH_POS_PID_CONFIG);
tap::algorithms::SmoothPid worldFramePitchTurretImuVelPid(
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

WorldFrameTurretImuCascadePidTurretController<tap::algorithms::transforms::Axis::PITCH>
    worldFramePitchTurretImuController(
        transformer.getWorldToTurretPitch(),
        drivers()->mcbLite.imu,
        engTurret.pitchMotor,
        worldFramePitchTurretImuPosPid,
        worldFramePitchTurretImuVelPid);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPid(world_rel_turret_imu::YAW_POS_PID_CONFIG);
tap::algorithms::SmoothPid worldFrameYawTurretImuVelPid(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

WorldFrameTurretImuCascadePidTurretController<tap::algorithms::transforms::Axis::YAW>
    worldFrameYawTurretImuController(
        transformer.getWorldToTurretPitch(),  // Pitch includes yaw
        drivers()->mcbLite.imu,
        engTurret.yawMotor,
        worldFrameYawTurretImuPosPid,
        worldFrameYawTurretImuVelPid);

BuzzerSubsystem buzzerSubsystem(drivers());

NoteSequenceCommand imuCalibrateSuccessBuzzCommand(
    buzzerSubsystem,
    IMU_CALIBRATE_SUCCESS_NOTES,
    IMU_CALIBRATE_SUCCESS_NOTE_LENGTH_MS);

NoteSequenceCommand imuCalibrateFailBuzzCommand(
    buzzerSubsystem,
    IMU_CALIBRATE_FAIL_NOTES,
    IMU_CALIBRATE_FAIL_NOTE_LENGTH_MS);

BinnedAlignmentCommand binnedAlignmentCommand(
    engTurret,
    lampreyEncoder,
    pulleyEncoder,
    *yawTurretMotor.getEncoder(),
    imu::ImuCalibrateCommand::DEFAULT_VELOCITY_ZERO_THRESHOLD,
    BINNED_ALIGNMENT_OFFSET,
    YAW_ALIGNMENT_OFFSET);

/* define client display / HUD related items --------------------------------*/
ClientDisplaySubsystem clientDisplay(drivers());
tap::communication::serial::RefSerialTransmitter refSerialTransmitter(drivers());

/* define commands ----------------------------------------------------------*/
HomingCommand cubeStorageHome(cubeStorage);
HomingCommand extensionHome(extensionSubsystem);

user::TurretUserWorldRelativeCommand turretUserWorldRelativeCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &engTurret,
    &chassisFrameYawTurretController,
    &chassisFramePitchTurretController,
    &chassisFrameYawTurretController,
    &chassisFramePitchTurretController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

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
    &chassisSubsystem);

WristControllerCommand wristControllerCommand(
    wristSubsystem,
    &drivers()->controlOperatorInterface,
    WRIST_THETA_1_SCALING_FACTOR,
    WRIST_THETA_2_SCALING_FACTOR,
    WRIST_THETA_3_SCALING_FACTOR);

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

ScorePositionCommand scorePositionCommand(extensionSubsystem, wristSubsystem);

SelectCubePositionCommand selectCubeAddPositionCommand(
    cubeStorage,
    true,
    transformer.getCubeStore1ToEndEffector(),
    transformer.getCubeStore2ToEndEffector());
SelectCubePositionCommand selectCubeRemovePositionCommand(
    cubeStorage,
    false,
    transformer.getCubeStore1ToEndEffector(),
    transformer.getCubeStore2ToEndEffector());
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

DigitalOutCommand endEffectorSuckOnCommand(leftSuckSubsystem, true);

DigitalOutCommand endEffectorSuckOffCommand(leftSuckSubsystem, false);

Transform IDENTITY_TRANSFORM;

inverse_kinematics::ManualIKCommand manualIKCommand(
    drivers()->controlOperatorInterface,
    transformer.getChassisToWorld(),
    IDENTITY_TRANSFORM,
    transformer.getWorldToEndEffector(),
    engTurret,
    extensionSubsystem,
    wristSubsystem,
    chassisFrameYawTurretController,
    chassisFramePitchTurretController);

SequentialCommand<3> storeCubeCommand(
    &selectCubeAddPositionCommand,
    &cubeStorageSuckOnCommand,
    // hand down
    // hand release cube
    // hand up
    &centerCubePosition);

SequentialCommand<3> removeCubeCommand(
    &selectCubeRemovePositionCommand,
    // hand suction on
    // hand down
    &cubeStorageSuckOffCommand,
    // hand up
    &centerCubePosition);

autotune::LampreyAutotuneCommand<36, Axis::YAW> lampreyAutotuneCommand(
    drivers(),
    {&engTurret,
     &yawEngTurretMotor,
     &chassisFrameYawTurretController,
     yawTurretMotor.isMotorInverted(),
     1,
     1},
    lampreyEncoder,
    &chassisSubsystem);

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

Trigger leftDownMidRightUp =
    (!TriggerHelpers::switchState(
         drivers(),
         Remote::Switch::LEFT_SWITCH,
         Remote::SwitchState::UP) &&
     TriggerHelpers::switchState(drivers(), Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP))
        .onTrue(&binnedAlignmentCommand);

// joint control mode
Trigger rightMid =
    TriggerHelpers::switchState(drivers(), Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID)
        .whileTrue(&turretUserWorldRelativeCommand)
        .whileTrue(&extensionManualControl)
        .whileTrue(&wristControllerCommand)
        .whileTrue(&chassisDriveCommand);

// IK mode
Trigger rightDown =
    TriggerHelpers::switchState(drivers(), Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN)
        .whileTrue(&manualIKCommand)
        .whileTrue(&chassisDriveCommand);

Trigger wheelDown =
    TriggerHelpers::channelGreaterThan(drivers(), Remote::Channel::WHEEL, 0.5f, false)
        .onTrue(&endEffectorSuckOnCommand);

Trigger wheelUp =
    (!TriggerHelpers::channelGreaterThan(drivers(), Remote::Channel::WHEEL, -0.5f, false))
        .onTrue(&endEffectorSuckOffCommand);

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    pulleyEncoder.initialize();
    lampreyEncoder.initialize();

    chassisSubsystem.initialize();
    engTurret.initialize();
    extensionSubsystem.initialize();
    wristSubsystem.initialize();
    cubeStorage.initialize();
    leftSuckSubsystem.initialize();
    rightSuckSubsystem.initialize();
    transformSubsystem.initialize();
    odometrySubsystem.initialize();
    parallelOmniOne.initialize();
    parallelOmniTwo.initialize();
    perpendicularOmni.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerEngineerSubsystems(aruwsrc::engineer::Drivers* drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassisSubsystem);
    drivers->commandScheduler.registerSubsystem(&extensionSubsystem);
    drivers->commandScheduler.registerSubsystem(&wristSubsystem);
    drivers->commandScheduler.registerSubsystem(&cubeStorage);
    drivers->commandScheduler.registerSubsystem(&leftSuckSubsystem);
    drivers->commandScheduler.registerSubsystem(&rightSuckSubsystem);
    drivers->commandScheduler.registerSubsystem(&engTurret);
    drivers->commandScheduler.registerSubsystem(&transformSubsystem);
    drivers->commandScheduler.registerSubsystem(&odometrySubsystem);
    // drivers->commandScheduler.registerSubsystem(&clientDisplay);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultEngineerCommands(aruwsrc::engineer::Drivers*)
{
    engTurret.setDefaultCommand(&turretUserWorldRelativeCommand);
    cubeStorage.setDefaultCommand(&cubeManualControl);

    // clientDisplay.setDefaultCommand(&clientDisplayCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startEngineerCommands(aruwsrc::engineer::Drivers*) {}

/* register io mappings here ------------------------------------------------*/
void registerEngineerIoMappings(aruwsrc::engineer::Drivers*) {}
}  // namespace control
}  // namespace aruwsrc

std::vector<aruwsrc::control::autotune::TurretAutotuneInterface*> getAutotuneCommands()
{
    static std::vector<aruwsrc::control::autotune::TurretAutotuneInterface*> commands = {
        &aruwsrc::control::lampreyAutotuneCommand};
    return commands;
}

namespace aruwsrc::engineer
{
void initSubsystemCommands(aruwsrc::engineer::Drivers* drivers)
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
