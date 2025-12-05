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
#include "aruwsrc/control/chassis/mecanum_chassis_subsystem.hpp"
#include "aruwsrc/control/client-display/client_display_command.hpp"
#include "aruwsrc/control/client-display/client_display_subsystem.hpp"
#include "aruwsrc/control/cycle_state_command_mapping.hpp"
#include "aruwsrc/control/joint/homing/homing_command.hpp"
#include "aruwsrc/control/joint/homing/trigger/limit_switch_trigger.hpp"
#include "aruwsrc/control/joint/homing/trigger_homed_dual_joint_subsystem.hpp"
#include "aruwsrc/control/joint/joint_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/engineer/cubelift_switch_command.hpp"
#include "aruwsrc/robot/engineer/digital_out_command.hpp"
#include "aruwsrc/robot/engineer/digital_out_subsystem.hpp"
#include "aruwsrc/robot/engineer/digital_out_toggle_command.hpp"
#include "aruwsrc/robot/engineer/engineer_cube_lift_constants.hpp"
#include "aruwsrc/robot/engineer/engineer_drivers.hpp"
#include "aruwsrc/robot/engineer/engineer_gantry_constants.hpp"
#include "aruwsrc/robot/engineer/engineer_setpoint_constants.hpp"
#include "aruwsrc/robot/engineer/engineer_wrist_constants.hpp"
#include "aruwsrc/robot/engineer/score_position_command.hpp"
#include "aruwsrc/robot/engineer/setpoint_move_manual_command.hpp"
#include "aruwsrc/robot/engineer/setpoint_move_position_command.hpp"
#include "aruwsrc/robot/engineer/sliders_indicator.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_controller_command.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_move_position_command.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_setpoints_command.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_subsystem.hpp"

#include "aruwsrc/robot/engineer/turret/engineer_turret_subsystem.hpp"
// #include "aruwsrc/robot/engineer/turret/constants/engineer_turret_constants.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/algorithms/odometry/otto_chassis_world_yaw_observer.hpp"
#include "aruwsrc/control/chassis/chassis_autorotate_command.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"

#include "aruwsrc/control/imu/imu_calibrate_command.hpp"

// check which of these r important
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"

#include "aruwsrc/control/turret/user/turret_quick_turn_command.hpp"
#include "aruwsrc/control/turret/user/turret_user_world_relative_command.hpp"

#include "aruwsrc/control/buzzer/note_sequence_command.hpp"
#include "aruwsrc/control/buzzer/note_sequences.hpp"
#include "aruwsrc/control/buzzer/buzzer_subsystem.hpp"
#include "aruwsrc/control/chassis/x_drive_chassis_subsystem.hpp"
#include "aruwsrc/control/governor/imu_calibrate_done_governor.hpp"

using namespace aruwsrc::control::client_display;
using namespace aruwsrc::control::client_display::indicators;
using namespace aruwsrc::control::joint;
using namespace aruwsrc::control::joint::homing;
using namespace aruwsrc::control::joint::homing::trigger;
using namespace aruwsrc::engineer;
using namespace aruwsrc::engineer::wrist;
using namespace tap::control;
using namespace tap::gpio;

using tap::communication::serial::Remote;
using tap::control::CommandMapper;

using namespace aruwsrc::control::turret;
using namespace aruwsrc::algorithms::odometry;
using namespace aruwsrc::control::buzzer;

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

EngineerTurretSubsystem turret(
    drivers(),
    &pitchTurretMotor,
    &yawTurretMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG,
    &getTurretMCBCanComm());

aruwsrc::algorithms::odometry::OttoChassisWorldYawObserver yawObserver(turret);

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

tap::motor::DjiMotor cubeLiftMotor(
    drivers(),
    CUBE_LIFT_MOTOR_ID,
    CUBE_LIFT_MOTOR_CAN_BUS,
    true,
    "Cube Lift Motor",
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
// chassis::MecanumChassisSubsystem mechanumChassis( 
//     drivers(),
//     &currentSensor,
//     &voltageSensor,
//     leftFrontChassisMotor,
//     leftBackChassisMotor,
//     rightFrontChassisMotor,
//     rightBackChassisMotor,
//     aruwsrc::control::chassis::WHEEL_VELOCITY_PID_CONFIG);

// x drive chassis now i think?
XDriveChassisSubsystem chassis(
    drivers(),
    &voltageCurrentSensor,
    &voltageCurrentSensor,
    leftFrontChassisMotor,
    leftBackChassisMotor,
    rightFrontChassisMotor,
    rightBackChassisMotor,
    aruwsrc::control::chassis::WHEEL_VELOCITY_PID_CONFIG);


// this could be useful i think

aruwsrc::control::chassis::ChassisAutorotateCommand chassisAutorotateCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis,
    &turret.yawMotor,
    aruwsrc::control::chassis::ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180);

algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController(
    turret.pitchMotor,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
    turret.yawMotor,
    chassis_rel::YAW_PID_CONFIG);

BuzzerSubsystem buzzer(drivers());

NoteSequenceCommand imuCalibrateSuccessBuzzCommand(
    buzzer,
    IMU_CALIBRATE_SUCCESS_NOTES,
    IMU_CALIBRATE_SUCCESS_NOTE_LENGTH_MS);

NoteSequenceCommand imuCalibrateFailBuzzCommand(
    buzzer,
    IMU_CALIBRATE_FAIL_NOTES,
    IMU_CALIBRATE_FAIL_NOTE_LENGTH_MS);

imu::ImuCalibrateCommand imuCalibrateCommand(
    drivers(),
    {{
        &getTurretMCBCanComm(),
        &turret,
        &chassisFrameYawTurretController,
        &chassisFramePitchTurretController,
        true,
    }},
    &chassis,
    imu::ImuCalibrateCommand::DEFAULT_VELOCITY_ZERO_THRESHOLD,
    imu::ImuCalibrateCommand::DEFAULT_POSITION_ZERO_THRESHOLD,
    &imuCalibrateSuccessBuzzCommand,
    &imuCalibrateFailBuzzCommand,
    &odometrySubsystem,
    // {&drivers()->ism330});
    {&drivers()->mpu6500});

IMUCalibrateDoneGovernor imuCalibrateDoneGovernor(drivers(), imuCalibrateCommand);

TriggerHomedJointSubsystem cubeLift(drivers(), cubeLiftMotor, cubeLiftTrigger, CUBE_LIFT_CONFIG);

WristSubsystem wristSubsystem(
    drivers(),
    wristLeftMotor,
    wristRightMotor,
    wristPitchEncoder,
    wristYawEncoder,
    WRIST_CONFIG);

TriggerHomedDualJointSubsystem gantryLiftSubsystem(
    drivers(),
    gantryLiftLeftMotor,
    gantryLiftRightMotor,
    gantryLiftTrigger,
    GANTRY_LIFT_ALIGN_PID_CONFIG,
    GANTRY_LIFT_CONFIG);

TriggerHomedJointSubsystem gantryExtensionSubsystem(
    drivers(),
    gantryExtensionMotor,
    gantryExtensionTrigger,
    GANTRY_EXTENSION_CONFIG);

JointSubsystem wristRollSubsystem(drivers(), wristRollMotor, WRIST_ROLL_CONFIG);

DigitalOutSubsystem suckSubsystem(
    drivers(),
    drivers()->digital,
    tap::gpio::Digital::OutputPin::Y,
    true);

DigitalOutSubsystem releaseSubsystem(
    drivers(),
    drivers()->digital,
    tap::gpio::Digital::OutputPin::Z,
    false);

/* define client display / HUD related items --------------------------------*/
ClientDisplaySubsystem clientDisplay(drivers());
tap::communication::serial::RefSerialTransmitter refSerialTransmitter(drivers());

SlidersIndicator slidersIndicator(
    refSerialTransmitter,
    gantryLiftSubsystem,
    gantryExtensionSubsystem,
    cubeLift,
    wristSubsystem,
    WRIST_CONFIG);

std::vector<HudIndicator *> hudIndicators = {&slidersIndicator};

aruwsrc::control::client_display::ClientDisplayCommand clientDisplayCommand(
    *drivers(),
    clientDisplay,
    hudIndicators);

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

aruwsrc::control::chassis::ChassisDriveCommand chassisDriveCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &mechanumChassis);

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

DigitalOutCommand suckOffCommand(suckSubsystem, false);
DigitalOutCommand suckOnCommand(suckSubsystem, true);
DigitalOutCommand releaseOffCommand(releaseSubsystem, false);
DigitalOutCommand releaseOnCommand(releaseSubsystem, true);
DigitalOutToggleCommand suctionToggleCommand(suckSubsystem, releaseSubsystem);

// commands here for sequences, but setpoints never tuned
SetpointMovePositionCommand liftUpCommand(gantryLiftSubsystem, 2);
SetpointMovePositionCommand liftDownCommand(gantryLiftSubsystem, 2);
SetpointMovePositionCommand gantryRetractCommand(gantryExtensionSubsystem, 2);
SetpointMovePositionCommand gantryExtendCommand(gantryExtensionSubsystem, 2);
// never tested
CubeliftSwitchCommand cubeLiftSwitchUpCommand(cubeLift, true);
CubeliftSwitchCommand cubeLiftSwitchDownCommand(cubeLift, false);

// sequences planned, but never finished and tuned
SequentialCommand<10> storeCubeCommand(std::array<Command *, 10>{
    {&liftUpCommand,
     &gantryRetractCommand,
     &wristFoldInCommand,
     &liftDownCommand,
     &suckOffCommand,
     &releaseOnCommand,
     &gantryExtendCommand,
     &liftUpCommand,
     &gantryRetractCommand,
     &cubeLiftSwitchDownCommand}});
SequentialCommand<10> retrieveCubeCommand(std::array<Command *, 10>{
    {&liftDownCommand,
     &gantryExtendCommand,
     &wristFoldInCommand,
     &gantryRetractCommand,
     &suckOnCommand,
     &releaseOffCommand,
     &liftUpCommand,
     &wristFoldOutCommand,
     &liftDownCommand,
     &cubeLiftSwitchUpCommand}});

// commands for pickup/scoring positions
SetpointMovePositionCommand gantryOut(gantryExtensionSubsystem, GANTRY_EXTENSION_SCORE);
SetpointMovePositionCommand gantryIn(gantryExtensionSubsystem, GANTRY_EXTENSION_PICKUP);
SetpointMovePositionCommand liftScore(gantryLiftSubsystem, GANTRY_LIFT_SCORE);
SetpointMovePositionCommand liftPickup(gantryLiftSubsystem, GANTRY_LIFT_PICKUP);
WristMovePositionCommand wristDown(
    wristSubsystem,
    WRIST_PITCH_PICKUP,
    WRIST_YAW_PICKUP);  // tuned to align for better suction
WristMovePositionCommand wristOut(wristSubsystem, WRIST_PITCH_SCORE, WRIST_YAW_SCORE);

ScorePositionCommand scorePositionCommand(
    gantryLiftSubsystem,
    wristSubsystem,
    wristRollSubsystem);  // TODO: test that this works

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

tap::control::PressCommandMapping leftUp(
    drivers(),
    {&cubeLiftHome, &gantryLiftHome, &gantryExtensionHome},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

tap::control::HoldCommandMapping rightMid(
    drivers(),
    {&suckOffCommand, &releaseOffCommand},
    tap::control::RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

tap::control::HoldCommandMapping rightDown(
    drivers(),
    {&suckOnCommand, &releaseOnCommand},
    tap::control::RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));

tap::control::PressCommandMapping suctionToggle(
    drivers(),
    {&suctionToggleCommand},
    RemoteMapState({Remote::Key::F}));

tap::control::PressCommandMapping vPressed(
    drivers(),
    {&gantryOut, &liftPickup, &wristDown},
    RemoteMapState({Remote::Key::V}));

tap::control::PressCommandMapping bPressed(
    drivers(),
    {&gantryIn, &liftScore, &wristOut},
    RemoteMapState({Remote::Key::B}));

// following commands never tested, and still need to get working
tap::control::PressCommandMapping storeCube(
    drivers(),
    {&storeCubeCommand},
    RemoteMapState({Remote::Key::Z}, {Remote::Key::SHIFT}));
tap::control::PressCommandMapping retrieveCube(
    drivers(),
    {&retrieveCubeCommand},
    RemoteMapState({Remote::Key::X}, {Remote::Key::SHIFT}));

tap::control::PressCommandMapping cubeLiftUp(
    drivers(),
    {&cubeLiftSwitchUpCommand},
    RemoteMapState({Remote::Key::Z, Remote::Key::SHIFT}));
tap::control::PressCommandMapping cubeLiftDown(
    drivers(),
    {&cubeLiftSwitchUpCommand},
    RemoteMapState({Remote::Key::X, Remote::Key::SHIFT}));

tap::control::PressCommandMapping cyclePositions(
    drivers(),
    {&scorePositionCommand},
    RemoteMapState({Remote::Key::C}));

CycleStateCommandMapping<ScorePositions, 3, ScorePositionCommand> cPressed(
    drivers(),
    RemoteMapState({Remote::Key::C}, {Remote::Key::SHIFT}),
    ScorePositions::three,
    &scorePositionCommand,
    &ScorePositionCommand::cyclePositions,
    RemoteMapState({Remote::Key::C, Remote::Key::SHIFT}));

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    mechanumChassis.initialize();
    gantryLiftSubsystem.initialize();
    gantryExtensionSubsystem.initialize();
    wristRollSubsystem.initialize();
    wristSubsystem.initialize();
    cubeLift.initialize();
    suckSubsystem.initialize();
    releaseSubsystem.initialize();
    // clientDicsplay.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerEngineerSubsystems(aruwsrc::engineer::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&mechanumChassis);
    drivers->commandScheduler.registerSubsystem(&gantryLiftSubsystem);
    drivers->commandScheduler.registerSubsystem(&gantryExtensionSubsystem);
    drivers->commandScheduler.registerSubsystem(&wristRollSubsystem);
    drivers->commandScheduler.registerSubsystem(&wristSubsystem);
    drivers->commandScheduler.registerSubsystem(&cubeLift);
    drivers->commandScheduler.registerSubsystem(&suckSubsystem);
    drivers->commandScheduler.registerSubsystem(&releaseSubsystem);
    // drivers->commandScheduler.registerSubsystem(&clientDisplay);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultEngineerCommands(aruwsrc::engineer::Drivers *)
{
    mechanumChassis.setDefaultCommand(&chassisDriveCommand);
    gantryLiftSubsystem.setDefaultCommand(&gantryLiftManualControl);
    gantryExtensionSubsystem.setDefaultCommand(&gantryExtensionManualControl);
    wristSubsystem.setDefaultCommand(&wristControllerCommand);
    wristRollSubsystem.setDefaultCommand(&wristControllerCommand);
    cubeLift.setDefaultCommand(&cubeManualControl);

    // clientDisplay.setDefaultCommand(&clientDisplayCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startEngineerCommands(aruwsrc::engineer::Drivers *) {}

/* register io mappings here ------------------------------------------------*/
void registerEngineerIoMappings(aruwsrc::engineer::Drivers *drivers)
{
    drivers->commandMapper.addMap(&suctionToggle);
    // drivers->commandMapper.addMap(&cubeLiftUp);
    // drivers->commandMapper.addMap(&cubeLiftDown);
    // drivers->commandMapper.addMap(&storeCube);
    // drivers->commandMapper.addMap(&retrieveCube);
    // drivers->commandMapper.addMap(&cyclePositions);
    // drivers->commandMapper.addMap(&cPressed);
    drivers->commandMapper.addMap(&leftUp);
    drivers->commandMapper.addMap(&rightMid);
    drivers->commandMapper.addMap(&rightDown);
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
