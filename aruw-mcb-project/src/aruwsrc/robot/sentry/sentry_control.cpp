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
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/sequential_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/double_dji_motor.hpp"

#include "aruwsrc/algorithms/odometry/wheel_ekf_odometry_2d_subsystem.hpp"
#include "aruwsrc/communication/can/aruw_analog_sensor.hpp"
#include "aruwsrc/communication/can/aruw_voltage_current_sensor.hpp"
#include "aruwsrc/communication/sensors/encoder/analog_sensor_encoder.hpp"
#include "aruwsrc/control/agitator/constant_fire_rate_agitator_command.hpp"
#include "aruwsrc/control/agitator/constant_velocity_agitator_command.hpp"
#include "aruwsrc/control/agitator/constants/agitator_constants.hpp"
#include "aruwsrc/control/agitator/unjam_spoke_agitator_command.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/aruco/aruco_reset_subsystem.hpp"
#include "aruwsrc/control/auto-aim/auto_aim_fire_rate_reselection_manager.hpp"
#include "aruwsrc/control/autotune/gravity_autotune.hpp"
#include "aruwsrc/control/buzzer/buzzer_subsystem.hpp"
#include "aruwsrc/control/buzzer/note_sequence_command.hpp"
#include "aruwsrc/control/buzzer/note_sequences.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "aruwsrc/control/chassis/half_swerve_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/sentry/auto_nav_beyblade_command.hpp"
#include "aruwsrc/control/chassis/swerve_module.hpp"
#include "aruwsrc/control/chassis/swerve_module_config.hpp"
#include "aruwsrc/control/chassis/x_drive_chassis_subsystem.hpp"
#include "aruwsrc/control/client-display/client_display_command.hpp"
#include "aruwsrc/control/client-display/client_display_subsystem.hpp"
#include "aruwsrc/control/client-display/indicators/circle_crosshair.hpp"
#include "aruwsrc/control/client-display/indicators/image_indicator.hpp"
#include "aruwsrc/control/governor/fire_rate_limit_governor.hpp"
#include "aruwsrc/control/governor/friction_wheels_on_governor.hpp"
#include "aruwsrc/control/governor/heat_limit_governor.hpp"
#include "aruwsrc/control/governor/imu_not_calibrated_governor.hpp"
#include "aruwsrc/control/governor/match_running_governor.hpp"
#include "aruwsrc/control/governor/ref_system_projectile_launched_governor.hpp"
#include "aruwsrc/control/launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "aruwsrc/control/launcher/launcher_constants.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"
#include "aruwsrc/control/turret/algorithms/turret_spring_compensation.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/yaw_turret_subsystem.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/sentry/algorithms/odometry/sentry_chassis_world_yaw_observer.hpp"
#include "aruwsrc/robot/sentry/algorithms/odometry/sentry_transform_adapter.hpp"
#include "aruwsrc/robot/sentry/algorithms/odometry/sentry_transform_subsystem.hpp"
#include "aruwsrc/robot/sentry/algorithms/sentry_ballistics_solver.hpp"
#include "aruwsrc/robot/sentry/chassis/sentry_beyblade_command.hpp"
#include "aruwsrc/robot/sentry/chassis/sentry_manual_drive_command.hpp"
#include "aruwsrc/robot/sentry/sentry_control_operator_interface.hpp"
#include "aruwsrc/robot/sentry/sentry_imu_calibrate_command.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_constants.hpp"
#include "aruwsrc/robot/sentry/turret/cv/sentry_auto_aim_launch_timer.hpp"
#include "aruwsrc/robot/sentry/turret/cv/sentry_minor_cv_on_target_governor.hpp"
#include "aruwsrc/robot/sentry/turret/cv/sentry_turret_cv_command.hpp"
#include "aruwsrc/robot/sentry/turret/sentry_turret_major_world_relative_yaw_controller.hpp"
#include "aruwsrc/robot/sentry/turret/sentry_turret_minor_subsystem.hpp"
#include "aruwsrc/robot/sentry/turret/turret_major_control_command.hpp"
#include "aruwsrc/robot/sentry/turret/turret_minor_control_command.hpp"

using namespace tap::algorithms;
using namespace tap::control;
using namespace tap::communication::serial;
using namespace tap::control::governor;
using namespace tap::control::setpoint;

using namespace aruwsrc::control::agitator;
using namespace aruwsrc::control;
using namespace aruwsrc::control::agitator;
using namespace aruwsrc::control::auto_aim;
using namespace aruwsrc::control::buzzer;
using namespace aruwsrc::control::client_display;
using namespace aruwsrc::control::client_display::indicators;
using namespace aruwsrc::control::governor;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::control::turret::algorithms;
using namespace aruwsrc::sentry;
using namespace aruwsrc::control::chassis;
using namespace aruwsrc::sentry::chassis;
using namespace aruwsrc::sentry::algorithms;
using namespace aruwsrc::sentry::algorithms::odometry;
using namespace aruwsrc::sentry::turret;
using namespace aruwsrc::sentry::turret::cv;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace sentry_control
{
MatchRunningGovernor matchRunningGovernor(drivers()->refSerial);

aruwsrc::communication::can::AruwAnalogSensor turretMajorYawAnalogSensor(
    drivers(),
    turretMajor::YAW_ANALOG_SENSOR_CAN_BUS,
    turretMajor::YAW_ANALOG_SENSOR_CAN_ID);

aruwsrc::communication::sensors::encoder::AnalogSensorEncoder::Calibration
    turretMajorYawAnalogCalibration{
        .rawMin = turretMajor::YAW_ANALOG_RAW_MIN,
        .rawMax = turretMajor::YAW_ANALOG_RAW_MAX,
        .rawZero = turretMajor::YAW_ANALOG_RAW_ZERO,
        .outputRangeRadians = turretMajor::YAW_ANALOG_OUTPUT_RANGE_RADIANS,
    };

aruwsrc::communication::sensors::encoder::AnalogSensorEncoder turretMajorYawAnalogEncoder(
    &turretMajorYawAnalogSensor,
    turretMajor::YAW_ANALOG_SENSOR_CHANNEL == 0
        ? aruwsrc::communication::sensors::encoder::AnalogSensorEncoder::Channel::AI0
        : aruwsrc::communication::sensors::encoder::AnalogSensorEncoder::Channel::AI1,
    turretMajorYawAnalogCalibration,
    turretMajor::YAW_ANALOG_SENSOR_INVERTED);

tap::motor::DjiMotor turretMajorYawMotor(
    drivers(),
    tap::motor::MOTOR5,
    turretMajor::CAN_BUS_MOTOR,
    false,
    "Major Yaw Turret",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508 *(27.0f / 95.0f),  // pulley ratio
    turretMajor::YAW_MOTOR_CONFIG.startEncoderValue,
    &turretMajorYawAnalogEncoder);

struct TurretMinorMotors
{
    tap::motor::DjiMotor yawMotor;
    tap::motor::DjiMotor pitchMotor;
    TurretMotorConfig yawMotorConfig;
    TurretMotorConfig pitchMotorConfig;
};

TurretMinorMotors turretWidowMotors{
    .yawMotor = tap::motor::DjiMotor(
        drivers(),
        turretWidow::YAW_MOTOR_ID,
        turretWidow::CAN_BUS_MOTORS,
        false,
        "Widow Minor Yaw Turret",
        true,
        1.0,
        turretWidow::YAW_MOTOR_CONFIG.startEncoderValue),

    .pitchMotor = tap::motor::DjiMotor(
        drivers(),
        turretWidow::PITCH_MOTOR_ID,
        turretWidow::CAN_BUS_MOTORS,
        true,
        "Widow Minor Pitch Turret",
        true,
        1.0,
        turretWidow::PITCH_MOTOR_CONFIG.startEncoderValue),

    .yawMotorConfig = turretWidow::YAW_MOTOR_CONFIG,
    .pitchMotorConfig = turretWidow::PITCH_MOTOR_CONFIG

};

inline aruwsrc::communication::can::TurretMCBCanComm &getTurretMCBCanCommWidow()
{
    return drivers()->turretMCBCanCommBus1;
}
// the other one
inline aruwsrc::communication::can::TurretMCBCanComm &getChassisTurretMCBCanComm()
{
    // return (&getTurretMCBCanCommWidow() == &drivers()->turretMCBCanCommBus1)
    //            ? drivers()->turretMCBCanCommBus2
    //            : drivers()->turretMCBCanCommBus1;
    return drivers()->turretMCBCanCommBus2;
}

// /* define subsystems --------------------------------------------------------*/
BuzzerSubsystem buzzer(drivers());

YawTurretSubsystem turretMajor(*drivers(), turretMajorYawMotor, turretMajor::YAW_MOTOR_CONFIG);

SentryTurretMinorSubsystem turretWidow(
    *drivers(),
    turretWidowMotors.pitchMotor,
    turretWidowMotors.yawMotor,
    turretWidowMotors.pitchMotorConfig,
    turretWidowMotors.yawMotorConfig,
    &drivers()->turretMCBCanCommBus1,  // @todo: figure out how to put this in config
    turretWidow::turretID);

SentryChassisWorldYawObserver chassisYawObserver(drivers()->turretMajorImu, turretMajor);

// Turret Compensators
TurretGravitationalForceOffset turretGravityCompensation(TURRET_GRAVITY_CONFIG);
TurretSpringForceOffset turretSpringCompensation(
    TURRET_SPRING_CONFIG,
    turretWidowMotors.pitchMotor.isMotorInverted());

struct TurretMinorChassisControllers
{
    ChassisFrameTurretController<Axis::PITCH> pitchController;
    ChassisFrameTurretController<Axis::YAW> yawController;
};

// @todo make controllers part of subsystem
TurretMinorChassisControllers turretWidowChassisControllers{
    .pitchController = ChassisFrameTurretController<Axis::PITCH>(
        turretWidow.pitchMotor,
        minorPidConfigs::PITCH_PID_CONFIG_CHASSIS_FRAME,
        {&turretGravityCompensation, &turretSpringCompensation}),
    .yawController = ChassisFrameTurretController<Axis::YAW>(
        turretWidow.yawMotor,
        minorPidConfigs::YAW_PID_CONFIG_CHASSIS_FRAME),
};

DjiMotor rightFrontMotor(
    drivers(),
    MOTOR1,
    tap::can::CanBus::CAN_BUS2,
    false,
    "Right Front Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

DjiMotor leftFrontMotor(
    drivers(),
    MOTOR2,
    tap::can::CanBus::CAN_BUS2,
    false,
    "Left Front Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

DjiMotor leftBackMotor(
    drivers(),
    MOTOR3,
    tap::can::CanBus::CAN_BUS2,
    false,
    "Left Back Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

DjiMotor rightBackMotor(
    drivers(),
    MOTOR4,
    tap::can::CanBus::CAN_BUS2,
    false,
    "Right Back Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

aruwsrc::communication::can::AruwVoltageCurrentSensor voltageCurrentSensor(
    drivers(),
    tap::can::CanBus::CAN_BUS2);

aruwsrc::control::chassis::XDriveChassisSubsystem chassis(
    drivers(),
    &voltageCurrentSensor,
    &voltageCurrentSensor,
    leftFrontMotor,
    leftBackMotor,
    rightFrontMotor,
    rightBackMotor,
    {.kp = 5.0f, .ki = 0.0f, .kd = 0.0f, .maxOutput = 16000.0f, .errDeadzone = 100.0f},
    WHEEL_RADIUS,
    WHEELBASE_RADIUS,
    &drivers()->capacitorBank);

const tap::motor::DjiMotor *sentryChassisMotorsForEkf[4] = {
    &leftFrontMotor,
    &rightFrontMotor,
    &leftBackMotor,
    &rightBackMotor};

aruwsrc::algorithms::odometry::WheelEKFOdometry2DSubsystem odometrySubsystem(
    *drivers(),
    sentryChassisMotorsForEkf,
    chassisYawObserver,
    getChassisTurretMCBCanComm(),
    modm::Vector2f(INITIAL_CHASSIS_POSITION_X, INITIAL_CHASSIS_POSITION_Y));

SentryTransforms transformer(
    odometrySubsystem,
    turretMajor,
    turretWidow,
    getTurretMCBCanCommWidow(),
    {
        .turretMinorOffset = TURRET_MINOR_OFFSET,
        .imuSyncConfig = IMU_SYNC_PID_CONFIG,
    });

SentryTransformSubystem transformerSubsystem(*drivers(), transformer);
SentryTransformAdapter transformAdapter(transformer);

aruwsrc::control::aruco::ArucoResetSubsystem arucoResetSubsystem(
    drivers(),
    drivers()->visionCoprocessor,
    odometrySubsystem,
    transformAdapter);

aruwsrc::control::cap_bank::CapBankSubsystem capBankSubsystem(drivers(), drivers()->capacitorBank);

aruwsrc::control::chassis::ChassisAutoNavController autoNavController(
    *drivers(),
    chassis,
    &transformAdapter,
    aruwsrc::control::chassis::BEYBLADE_CONFIG,
    capBankSubsystem,
    0.15f,
    1000.0f);

SmoothPid turretMajorYawPosPid(turretMajor::worldFrameCascadeController::YAW_POS_PID_CONFIG);
SmoothPid turretMajorYawVelPid(turretMajor::worldFrameCascadeController::YAW_VEL_PID_CONFIG);

struct TurretMinorWorldControllers
{
    WorldFrameTurretImuCascadePidTurretController<Axis::PITCH> pitchController;
    WorldFrameTurretImuCascadePidTurretController<Axis::YAW> yawController;
};

// // @todo surely there's a better way to construct this
SmoothPid turretWidowWorldPitchVelPid(minorPidConfigs::PITCH_PID_CONFIG_WORLD_FRAME_VEL);
SmoothPid turretWidowWorldPitchPosPid(minorPidConfigs::PITCH_PID_CONFIG_WORLD_FRAME_POS);
SmoothPid turretWidowWorldYawVelPid(minorPidConfigs::LEFT_YAW_PID_CONFIG_WORLD_FRAME_VEL);
SmoothPid turretWidowWorldYawPosPid(minorPidConfigs::YAW_PID_CONFIG_WORLD_FRAME_POS);

TurretMinorWorldControllers turretWidowWorldControllers{
    .pitchController = WorldFrameTurretImuCascadePidTurretController<Axis::PITCH>(
        transformer.getWorldToTurretWidow(),
        getTurretMCBCanCommWidow(),
        turretWidow.pitchMotor,
        turretWidowWorldPitchPosPid,
        turretWidowWorldPitchVelPid,
        {&turretGravityCompensation, &turretSpringCompensation}),

    .yawController = WorldFrameTurretImuCascadePidTurretController<Axis::YAW>(
        transformer.getWorldToTurretWidow(),
        getTurretMCBCanCommWidow(),
        turretWidow.yawMotor,
        turretWidowWorldYawPosPid,
        turretWidowWorldYawVelPid)

};

TurretMajorWorldFrameController turretMajorWorldYawController(
    transformer.getWorldToTurretMajor(),
    chassis,
    turretMajor.getMutableMotor(),
    drivers()->turretMajorImu,
    turretWidow,
    turretMajorYawPosPid,
    turretMajorYawVelPid,
    turretMajor::MAX_VEL_ERROR_INPUT,
    turretMajor::TURRET_MINOR_TORQUE_RATIO,
    turretMajor::FEEDFORWARD_GAIN);

ChassisFrameTurretController<Axis::YAW> turretMajorChassisYawController(
    turretMajor.getMutableMotor(),
    turretMajor::chassisFrameController::YAW_PID_CONFIG);

// Friction Wheels
tap::motor::DjiMotor turretWidowFrictionWheelLeft(
    drivers(),
    aruwsrc::control::launcher::LEFT_MOTOR_ID,
    turretWidow::CAN_BUS_MOTORS,
    false,
    "Right flywheel");
tap::motor::DjiMotor turretWidowFrictionWheelRight(
    drivers(),
    aruwsrc::control::launcher::RIGHT_MOTOR_ID,
    turretWidow::CAN_BUS_MOTORS,
    true,
    "Right flywheel");
std::array<tap::motor::MotorInterface *, 2> turretWidowWheels = {
    &turretWidowFrictionWheelLeft,
    &turretWidowFrictionWheelRight};
aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE,
    2>
    turretWidowFrictionWheels(
        drivers(),
        turretWidowWheels,
        aruwsrc::control::launcher::WHEEL_CONFIG,
        aruwsrc::control::launcher::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT,
        turretWidow::barrelID);

// Agitators
VelocityAgitatorSubsystem turretWidowAgitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::turretWidow::AGITATOR_CONFIG);

// ballistics solvers
SentryBallisticsSolver turretWidowSolver(
    drivers()->visionCoprocessor,
    transformer,
    turretWidowFrictionWheels,
    turretMajor,
    turretWidow::DEFAULT_LAUNCH_SPEED,
    0.f,  // turret minor pitch offset
    TURRET_MINOR_OFFSET,
    turretWidow.getTurretID());

SentryAutoAimLaunchTimer autoAimLaunchTimerTurretWidow(
    aruwsrc::control::launcher::AGITATOR_TYPICAL_DELAY_MICROSECONDS,
    &drivers()->visionCoprocessor,
    &turretWidowSolver);

/* define commands ----------------------------------------------------------*/
aruwsrc::control::chassis::sentry::AutoNavBeybladeCommand autoNavBeybladeCommand(
    *drivers(),
    chassis,
    autoNavController,
    true);

TurretMajorSentryControlCommand majorManualCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    turretMajor,
    turretMajorWorldYawController,
    MAJOR_USER_YAW_INPUT_SCALAR);

TurretMinorSentryControlCommand turretWidowManualCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    turretWidow,
    turretWidowWorldControllers.yawController,
    turretWidowWorldControllers.pitchController,
    MINOR_USER_YAW_INPUT_SCALAR,
    MINOR_USER_PITCH_INPUT_SCALAR);

// Chassis beyblade
SentryBeybladeCommand beybladeCommand(
    drivers(),
    &chassis,
    &turretMajor.getReadOnlyMotor(),
    drivers()->controlOperatorInterface,
    transformer.getWorldToChassis(),
    aruwsrc::control::chassis::BEYBLADE_CONFIG);

SentryManualDriveCommand chassisDriveCommand(
    drivers(),
    &(drivers()->controlOperatorInterface),
    &chassis);

NoteSequenceCommand imuNotCalibratedCommand(
    buzzer,
    IMU_NOT_CALIBRATED_NOTES,
    IMU_NOT_CALIBRATED_NOTE_LENGTH_MS);

NoteSequenceCommand imuCalibrateSuccessBuzzCommand(
    buzzer,
    IMU_CALIBRATE_SUCCESS_NOTES,
    IMU_CALIBRATE_SUCCESS_NOTE_LENGTH_MS);

NoteSequenceCommand imuCalibrateFailBuzzCommand(
    buzzer,
    IMU_CALIBRATE_FAIL_NOTES,
    IMU_CALIBRATE_FAIL_NOTE_LENGTH_MS);

SentryImuCalibrateCommand imuCalibrateCommand(
    drivers(),
    {
        {
            &getTurretMCBCanCommWidow(),
            &turretWidow,
            &turretWidowChassisControllers.yawController,
            &turretWidowChassisControllers.pitchController,
            true,
        },
    },
    turretMajor,
    turretMajorChassisYawController,
    chassis,
    chassisYawObserver,
    odometrySubsystem,
    drivers()->turretMajorImu,
    getChassisTurretMCBCanComm(),
    transformer,
    turretMajorYawAnalogEncoder,
    &imuCalibrateSuccessBuzzCommand,
    &imuCalibrateFailBuzzCommand);

ImuNotCalibratedGovernor imuNotCalibratedGovernor(drivers(), drivers()->mpu6500);

GovernorLimitedCommand<1> imuNotCalibratedCommandLimited(
    {&buzzer},
    imuNotCalibratedCommand,
    {&imuNotCalibratedGovernor});

autotune::GravityAutotuneCommand<9> gravityAutotuneCommandWidow(
    drivers(),
    {&turretWidow,
     &turretWidowChassisControllers.pitchController,
     turretWidowMotors.pitchMotor.isMotorInverted(),
     TURRET_WEIGHT_KG,
     TORQUE_TO_DESIRED_OUT});

SentryTurretCVCommand::TurretConfig turretWidowCVConfig(
    turretWidow,
    turretWidowWorldControllers.yawController,
    turretWidowWorldControllers.pitchController,
    turretWidowSolver);

SentryTurretCVCommand turretCVCommand(
    drivers()->visionCoprocessor,
    drivers()->plateHitTracker,
    turretMajor,
    turretMajorWorldYawController,
    turretWidowCVConfig,
    transformer);

// Widow shooting ======================

AutoAimFireRateReselectionManager fireRateReselectionManagerTurretWidow(
    *drivers(),
    drivers()->visionCoprocessor,
    drivers()->commandScheduler,
    turretCVCommand,
    turretWidow::turretID);

// spin friction wheels commands
aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand turretWidowFrictionWheelSpinCommand(
    drivers(),
    &turretWidowFrictionWheels,
    aruwsrc::control::launcher::LAUNCHER_SPEED,
    false,
    turretWidow::barrelID);

aruwsrc::control::launcher::
    FrictionWheelSpinRefLimitedCommand stopTurretWidowFrictionWheelSpinCommand(
        drivers(),
        &turretWidowFrictionWheels,
        0.0f,
        true,
        turretWidow::barrelID);

// Agitator commands (turret widow)
ConstantFireRateAgitatorCommand turretWidowRotateAgitator(
    turretWidowAgitator,
    ConstantFireRateAgitatorCommand::Config{
        constants::AGITATOR_ROTATE_CONFIG,
        constants::MANUAL_CONSTANT_FIRE_RATE_RPS,
        constants::AGITATOR_NUM_POCKETS,
        constants::MIN_CONSTANT_FIRE_RATE_RPM,
        &fireRateReselectionManagerTurretWidow});
UnjamSpokeAgitatorCommand turretWidowUnjamAgitator(
    turretWidowAgitator,
    constants::AGITATOR_UNJAM_CONFIG);
MoveUnjamIntegralComprisedCommand turretWidowRotateAndUnjamAgitator(
    *drivers(),
    turretWidowAgitator,
    turretWidowRotateAgitator,
    turretWidowUnjamAgitator);

FireRateLimitGovernor fireRateLimitGovernorTurretWidow(
    fireRateReselectionManagerTurretWidow,
    false);

// rotates agitator with heat limiting applied
HeatLimitGovernor heatLimitGovernorTurretWidow(
    *drivers(),
    turretWidow::barrelID,
    constants::HEAT_LIMIT_BUFFER);

// rotates agitator when aiming at target and within heat limit
SentryMinorCvOnTargetGovernor cvOnTargetGovernorTurretWidow(
    drivers(),
    drivers()->visionCoprocessor,
    turretCVCommand,
    autoAimLaunchTimerTurretWidow,
    SentryCvOnTargetGovernorMode::ON_TARGET_AND_GATED,
    turretWidow::turretID);

// Unused, causes incosnistent fire rates due to suspected ref delay.
// RefSystemProjectileLaunchedGovernor refSystemProjectileLaunchedGovernorTurretWidow(
//     drivers()->refSerial,
//     turretWidow::barrelID);

FrictionWheelsOnGovernor frictionWheelsOnGovernorTurretWidow(turretWidowFrictionWheels);

GovernorLimitedCommand<5> turretWidowRotateAndUnjamAgitatorWithHeatAndCVLimiting(
    {&turretWidowAgitator},
    turretWidowRotateAndUnjamAgitator,
    {&fireRateLimitGovernorTurretWidow,
     &heatLimitGovernorTurretWidow,
     &frictionWheelsOnGovernorTurretWidow,
     &cvOnTargetGovernorTurretWidow,
     &matchRunningGovernor});

GovernorLimitedCommand<2> turretWidowAgitatorManualSpin(
    {&turretWidowAgitator},
    turretWidowRotateAndUnjamAgitator,
    {&heatLimitGovernorTurretWidow, &frictionWheelsOnGovernorTurretWidow});

/* define client display / HUD related items --------------------------------*/

// This shit is currently banned by DJI, but left for a hopeful future
ClientDisplaySubsystem clientDisplay(drivers());
tap::communication::serial::RefSerialTransmitter refSerialTransmitter(drivers());

CircleCrosshair circleCrosshair(refSerialTransmitter);
ImageIndicator imageIndicator(refSerialTransmitter);

std::vector<HudIndicator *> indicators = {&imageIndicator, &circleCrosshair};

ClientDisplayCommand clientDisplayCommand(*drivers(), clientDisplay, indicators);

/* define command mappings --------------------------------------------------*/

HoldCommandMapping rightUp(
    drivers(),
    {&turretWidowFrictionWheelSpinCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

// auto nav + auto aim + cv gated fire
HoldRepeatCommandMapping leftUpRightUp(
    drivers(),
    {&autoNavBeybladeCommand, &turretCVCommand},
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::UP),
    true);

HoldRepeatCommandMapping leftUpRightUpAg(
    drivers(),
    {&turretWidowRotateAndUnjamAgitatorWithHeatAndCVLimiting},
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::UP),
    false);

// auto nav + auto aim
HoldCommandMapping leftUpRightMid(
    drivers(),
    {&autoNavBeybladeCommand, &turretCVCommand},
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::MID));

// imu calibrate
HoldCommandMapping leftUpRightDown(
    drivers(),
    {&imuCalibrateCommand},
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::DOWN));

// manual aim and shoot
HoldCommandMapping leftMidRightUp(
    drivers(),
    {&turretWidowManualCommand},
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::UP));

// manual aim and shoot
HoldRepeatCommandMapping leftMidRightUpAg(
    drivers(),
    {&turretWidowAgitatorManualSpin},
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::UP),
    false);

// auto drive & auto aim
HoldCommandMapping leftMidRightMid(
    drivers(),
    {&majorManualCommand, &turretWidowManualCommand, &autoNavBeybladeCommand},
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::MID));

// manual aim
HoldCommandMapping leftMidRightDown(
    drivers(),
    {
        &majorManualCommand,
        &turretWidowManualCommand,
    },
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::DOWN));

// manual drive, auto aim, cv-gated fire
HoldCommandMapping leftDownRightUp(
    drivers(),
    {&chassisDriveCommand, &turretCVCommand},
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::UP));

HoldRepeatCommandMapping leftDownRightUpAg(
    drivers(),
    {&turretWidowRotateAndUnjamAgitatorWithHeatAndCVLimiting},
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::UP),
    false);

// manual drive & auto aim
HoldCommandMapping leftDownRightMid(
    drivers(),
    {&chassisDriveCommand, &turretCVCommand},
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::MID));

// manual drive
HoldCommandMapping leftDownRightDown(
    drivers(),
    {&chassisDriveCommand},
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::DOWN));

// Restart HUD
PressCommandMapping bCtrlPressed(
    drivers(),
    {&clientDisplayCommand},
    RemoteMapState({Remote::Key::CTRL, Remote::Key::B}));

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());
/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    voltageCurrentSensor.initialize();
    turretMajorYawAnalogSensor.initialize();
    buzzer.initialize();
    chassis.initialize();
    turretWidow.initialize();
    turretMajor.initialize();
    odometrySubsystem.initialize();
    transformerSubsystem.initialize();
    arucoResetSubsystem.initialize();
    turretWidowFrictionWheels.initialize();

    turretWidowAgitator.initialize();

    clientDisplay.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerSentrySubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&buzzer);
    drivers->commandScheduler.registerSubsystem(&turretMajor);
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&turretWidow);
    drivers->commandScheduler.registerSubsystem(&odometrySubsystem);
    drivers->commandScheduler.registerSubsystem(&transformerSubsystem);
    drivers->commandScheduler.registerSubsystem(&arucoResetSubsystem);
    drivers->commandScheduler.registerSubsystem(&clientDisplay);

    drivers->commandScheduler.registerSubsystem(&turretWidowFrictionWheels);
    drivers->commandScheduler.registerSubsystem(&turretWidowAgitator);

    drivers->visionCoprocessor.attachTransformer(&transformAdapter);
    drivers->plateHitTracker.attachTransformer(&transformAdapter);
    drivers->visionCoprocessor.attachAutoNavController(&autoNavController);
    drivers->stateMachine.attachAutoNavController(&autoNavController);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSentryCommands(Drivers *)
{
    chassis.setDefaultCommand(&chassisDriveCommand);
    turretMajor.setDefaultCommand(&majorManualCommand);
    turretWidow.setDefaultCommand(&turretWidowManualCommand);

    turretWidowFrictionWheels.setDefaultCommand(&stopTurretWidowFrictionWheelSpinCommand);

    clientDisplay.setDefaultCommand(&clientDisplayCommand);

    buzzer.setDefaultCommand(&imuNotCalibratedCommandLimited);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSentryCommands(Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&imuCalibrateCommand);
    drivers->turretMajorImu.setMountingTransform(turretMajor::TURRET_MAJOR_IMU_MOUNTING_TRANSFORM);
}

/* register io mappings here ------------------------------------------------*/
void registerSentryIoMappings(Drivers *drivers)
{
    // commands with higher priority must be added later
    // friction wheels spin (separated due to dumb design in command mapper system)
    drivers->commandMapper.addMap(&rightUp);

    drivers->commandMapper.addMap(&leftDownRightMid);  // manual drive & auto aim
    drivers->commandMapper.addMap(&leftDownRightUp);   // manual drive, auto aim, gated-fire
    drivers->commandMapper.addMap(&leftDownRightUpAg);
    drivers->commandMapper.addMap(&leftDownRightDown);  // manual drive

    drivers->commandMapper.addMap(&leftMidRightUp);  // manual aim and shoot
    drivers->commandMapper.addMap(&leftMidRightUpAg);
    drivers->commandMapper.addMap(&leftMidRightMid);   // auto drive & auto aim
    drivers->commandMapper.addMap(&leftMidRightDown);  // manual aim

    drivers->commandMapper.addMap(&leftUpRightMid);  // auto nav + auto aim
    drivers->commandMapper.addMap(&leftUpRightUp);   // auto nav + auto aim + cv gated fire
    drivers->commandMapper.addMap(&leftUpRightUpAg);
    drivers->commandMapper.addMap(&leftUpRightDown);  // imu calibrate
}

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

#ifndef PLATFORM_HOSTED
std::vector<aruwsrc::control::autotune::TurretAutotuneInterface *> getAutotuneCommands()
{
    static std::vector<aruwsrc::control::autotune::TurretAutotuneInterface *> commands = {
        &sentry_control::gravityAutotuneCommandWidow};
    return commands;
}
#endif
// imu::ImuCalibrateCommand *getImuCalibrateCommand() { return
// &sentry_control::imuCalibrateCommand; }
