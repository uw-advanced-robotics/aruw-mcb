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
#include "tap/control/remote_map_state.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/algorithms/ballistics/cv_ballistics_solver.hpp"
#include "aruwsrc/algorithms/odometry/chassis_cf_odometry.hpp"
#include "aruwsrc/algorithms/odometry/wheel_ekf_odometry_2d_subsystem.hpp"
#include "aruwsrc/communication/can/aruw_voltage_current_sensor.hpp"
#include "aruwsrc/control/agitator/agitator_fan_command.hpp"
#include "aruwsrc/control/agitator/agitator_fan_subsystem.hpp"
#include "aruwsrc/control/agitator/constant_fire_rate_agitator_command.hpp"
#include "aruwsrc/control/agitator/constant_velocity_agitator_command.hpp"
#include "aruwsrc/control/agitator/constants/agitator_constants.hpp"
#include "aruwsrc/control/agitator/unjam_spoke_agitator_command.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/aruco/aruco_reset_subsystem.hpp"
#include "aruwsrc/control/auto-aim/auto_aim_fire_rate_reselection_manager.hpp"
#include "aruwsrc/control/autotune/freq_sweep_autotune.hpp"
#include "aruwsrc/control/autotune/gravity_autotune.hpp"
#include "aruwsrc/control/autotune/lamprey_autotune.hpp"
#include "aruwsrc/control/buzzer/buzzer_subsystem.hpp"
#include "aruwsrc/control/buzzer/note_sequence_command.hpp"
#include "aruwsrc/control/buzzer/note_sequences.hpp"
#include "aruwsrc/control/cap-bank/sentry_cap_bank_command.hpp"
#include "aruwsrc/control/chassis/auto_nav_command.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "aruwsrc/control/chassis/swerve_module.hpp"
#include "aruwsrc/control/chassis/swerve_module_config.hpp"
#include "aruwsrc/control/chassis/x_drive_chassis_subsystem.hpp"
#include "aruwsrc/control/client-display/client_display_command.hpp"
#include "aruwsrc/control/client-display/client_display_subsystem.hpp"
#include "aruwsrc/control/client-display/indicators/circle_crosshair.hpp"
#include "aruwsrc/control/client-display/indicators/image_indicator.hpp"
#include "aruwsrc/control/governor/cv_on_target_governor.hpp"
#include "aruwsrc/control/governor/fire_rate_limit_governor.hpp"
#include "aruwsrc/control/governor/friction_wheels_on_governor.hpp"
#include "aruwsrc/control/governor/heat_limit_governor.hpp"
#include "aruwsrc/control/governor/imu_not_calibrated_governor.hpp"
#include "aruwsrc/control/governor/match_running_governor.hpp"
#include "aruwsrc/control/launcher/friction_wheel_lut_autotune_command.hpp"
#include "aruwsrc/control/launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "aruwsrc/control/launcher/launcher_constants.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"
#include "aruwsrc/control/turret/algorithms/turret_spring_compensation.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_stos_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/yaw_turret_subsystem.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/sentry/algorithms/odometry/sentry_chassis_world_yaw_observer.hpp"
#include "aruwsrc/robot/sentry/algorithms/odometry/sentry_transform_adapter.hpp"
#include "aruwsrc/robot/sentry/algorithms/odometry/sentry_transform_subsystem.hpp"
#include "aruwsrc/robot/sentry/chassis/sentry_beyblade_command.hpp"
#include "aruwsrc/robot/sentry/chassis/sentry_manual_drive_command.hpp"
#include "aruwsrc/robot/sentry/sentry_control_operator_interface.hpp"
#include "aruwsrc/robot/sentry/sentry_imu_calibrate_command.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_constants.hpp"
#include "aruwsrc/robot/sentry/turret/cv/sentry_turret_cv_command.hpp"
#include "aruwsrc/robot/sentry/turret/sentry_turret_major_world_relative_yaw_controller.hpp"
#include "aruwsrc/robot/sentry/turret/sentry_turret_minor_subsystem.hpp"
#include "aruwsrc/robot/sentry/turret/turret_major_control_command.hpp"
#include "aruwsrc/robot/sentry/turret/turret_minor_control_command.hpp"

/// @TODO: test lamprey autotune's new 0 aligning -Aiden
/// @TODO: test binned alignment - Aiden
/// @TODO: clean up imu calibrate - Aiden

using namespace tap::algorithms;
using namespace tap::control;
using namespace tap::communication::serial;
using namespace tap::control::governor;
using namespace tap::control::setpoint;

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

aruwsrc::communication::sensors::encoder::LampreyEncoder turretMajorYawLamprey(
    drivers(),
    turretMajor::YAW_LAMPREY_CAN_ID,
    turretMajor::YAW_LAMPREY_CAN_BUS,
    turretMajor::LAMPREY_CALIBRATION_MAP,
    turretMajor::YAW_LAMPREY_INVERTED);

tap::encoder::CanEncoder turretMajorYawCanEncoder(
    drivers(),
    tap::encoder::CanEncoderId::ID0,
    tap::can::CanBus::CAN_BUS2,
    true);

tap::motor::DjiMotor turretMajorYawMotor(
    drivers(),
    tap::motor::MOTOR5,
    turretMajor::CAN_BUS_MOTOR,
    true,
    "Major Yaw Turret",
    false,
    turretMajor::PULLEY_RATIO,
    turretMajor::YAW_MOTOR_CONFIG.startEncoderValue);

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
        tap::motor::DjiMotorEncoder::GEAR_RATIO_GM6020,
        turretWidow::YAW_MOTOR_CONFIG.startEncoderValue),

    .pitchMotor = tap::motor::DjiMotor(
        drivers(),
        turretWidow::PITCH_MOTOR_ID,
        turretWidow::CAN_BUS_MOTORS,
        true,
        "Widow Minor Pitch Turret",
        true,
        tap::motor::DjiMotorEncoder::GEAR_RATIO_GM6020,
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
    return drivers()->turretMCBCanCommBus2;
}

inline tap::communication::sensors::imu::AbstractIMU &getTurretMajorImu()
{
    return drivers()->turretMajorImu;
}

// /* define subsystems --------------------------------------------------------*/
BuzzerSubsystem buzzer(drivers());

YawTurretSubsystem turretMajor(*drivers(), turretMajorYawMotor, turretMajor::YAW_MOTOR_CONFIG);

TurretMotor pitchTurretMotor(&turretWidowMotors.pitchMotor, turretWidowMotors.pitchMotorConfig);
TurretMotor yawTurretMotor(&turretWidowMotors.yawMotor, turretWidowMotors.yawMotorConfig);

SentryTurretMinorSubsystem turretWidow(
    *drivers(),
    pitchTurretMotor,
    yawTurretMotor,
    &drivers()->turretMCBCanCommBus1,  // @todo: figure out how to put this in config
    turretWidow::turretID);

SentryChassisWorldYawObserver chassisYawObserver(getTurretMajorImu(), turretMajor);

// Turret Compensators
TurretGravitationalForceOffset turretGravityCompensation(TURRET_GRAVITY_CONFIG);
TurretSpringForceOffset turretSpringCompensation(
    TURRET_SPRING_CONFIG,
    turretWidowMotors.pitchMotor.isMotorInverted());

struct TurretMinorChassisControllers
{
    ChassisFrameTurretController<transforms::Axis::PITCH> pitchController;
    ChassisFrameTurretController<transforms::Axis::YAW> yawController;
};

// @todo make controllers part of subsystem
TurretMinorChassisControllers turretWidowChassisControllers{
    .pitchController = ChassisFrameTurretController<transforms::Axis::PITCH>(
        turretWidow.pitchMotor,
        minorPidConfigs::PITCH_PID_CONFIG_CHASSIS_FRAME,
        {&turretGravityCompensation, &turretSpringCompensation}),
    .yawController = ChassisFrameTurretController<transforms::Axis::YAW>(
        turretWidow.yawMotor,
        minorPidConfigs::YAW_PID_CONFIG_CHASSIS_FRAME),
};

DjiMotor rightFrontMotor(
    drivers(),
    MOTOR1,
    tap::can::CanBus::CAN_BUS2,
    true,
    "Right Front Motor",
    false,
    CHASSIS_GEARBOX_RATIO);

DjiMotor leftFrontMotor(
    drivers(),
    MOTOR2,
    tap::can::CanBus::CAN_BUS2,
    true,
    "Left Front Motor",
    false,
    CHASSIS_GEARBOX_RATIO);

DjiMotor leftBackMotor(
    drivers(),
    MOTOR3,
    tap::can::CanBus::CAN_BUS2,
    true,
    "Left Back Motor",
    false,
    CHASSIS_GEARBOX_RATIO);

DjiMotor rightBackMotor(
    drivers(),
    MOTOR4,
    tap::can::CanBus::CAN_BUS2,
    true,
    "Right Back Motor",
    false,
    CHASSIS_GEARBOX_RATIO);

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
    WHEEL_VELOCITY_PID_CONFIG,
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
    modm::Vector2f(INITIAL_CHASSIS_POSITION_X, INITIAL_CHASSIS_POSITION_Y),
    &drivers()->rttTelemetry);

aruwsrc::algorithms::odometry::ChassisCFOdometry cfOdometrySubsystem(
    drivers(),
    chassis,
    chassisYawObserver,
    drivers()->turretMCBCanCommBus2,
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

aruwsrc::control::cap_bank::CapBankSubsystem capBankSubsystem(
    drivers(),
    drivers()->capacitorBank,
    voltageCurrentSensor);

// The sentry has no operator toggle, so its default command keeps the cap bank enabled.
aruwsrc::control::cap_bank::SentryCapBankCommand sentryCapBankCommand(drivers(), capBankSubsystem);

aruwsrc::control::chassis::ChassisAutoNavController autoNavController(
    *drivers(),
    chassis,
    transformAdapter.getWorldToChassis(),
    aruwsrc::control::chassis::BEYBLADE_CONFIG,
    &capBankSubsystem,
    CAP_BANK_SPRINT_ENERGY_THRESHOLD,
    CAP_BANK_SPRINT_TRANSLATIONAL_VELOCITY_THRESHOLD);

SmoothPid turretMajorYawPosPid(turretMajor::worldFrameCascadeController::YAW_POS_PID_CONFIG);
SmoothPid turretMajorYawVelPid(turretMajor::worldFrameCascadeController::YAW_VEL_PID_CONFIG);

struct TurretMinorWorldControllers
{
    WorldFrameTurretImuCascadePidTurretController<transforms::Axis::PITCH> pitchController;
    WorldFrameTurretImuSTOSTurretController<transforms::Axis::YAW> yawController;
};

SmoothPid turretWidowWorldPitchVelPid(minorPidConfigs::PITCH_PID_CONFIG_WORLD_FRAME_VEL);
SmoothPid turretWidowWorldPitchPosPid(minorPidConfigs::PITCH_PID_CONFIG_WORLD_FRAME_POS);
// SmoothPid turretWidowWorldYawVelPid(minorPidConfigs::MINOR_YAW_PID_CONFIG_WORLD_FRAME_VEL);
SmoothPid turretWidowWorldYawPosPid(minorPidConfigs::YAW_PID_CONFIG_WORLD_FRAME_POS);

TurretMinorWorldControllers turretWidowWorldControllers{
    .pitchController = WorldFrameTurretImuCascadePidTurretController<transforms::Axis::PITCH>(
        transformer.getWorldToTurretWidow(),
        getTurretMCBCanCommWidow(),
        turretWidow.pitchMotor,
        turretWidowWorldPitchPosPid,
        turretWidowWorldPitchVelPid,
        {&turretGravityCompensation, &turretSpringCompensation}),

    .yawController = WorldFrameTurretImuSTOSTurretController<transforms::Axis::YAW>(
        transformer.getWorldToTurretWidow(),
        getTurretMCBCanCommWidow(),
        turretWidow.yawMotor,
        turretWidow::turretWidowSTOSConstants,
        turretWidowWorldYawPosPid,
        turretWidow::turretWidowFeedforwardConstants)};

TurretMajorWorldFrameController turretMajorWorldYawController(
    transformer.getWorldToTurretMajor(),
    chassis,
    turretMajor.getMutableMotor(),
    getTurretMajorImu(),
    turretWidow,
    turretMajorYawPosPid,
    turretMajorYawVelPid,
    turretMajor::MAX_VEL_ERROR_INPUT);

ChassisFrameTurretController<transforms::Axis::YAW> turretMajorChassisYawController(
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

aruwsrc::algorithms::ballistics::CvBallisticsSolver ballisticsSolver(
    drivers()->visionCoprocessor,
    transformAdapter,
    turretWidowFrictionWheels,
    {
        .shotTimingEntryThreshold = SHOT_TIMING_ENTRY_THRESHOLD,
        .shotTimingExitThreshold = SHOT_TIMING_EXIT_THRESHOLD,
        .defaultLaunchSpeed = turretWidow::DEFAULT_LAUNCH_SPEED,
        .turretPitchOffset = 0,
        .minimumShotDelay = aruwsrc::control::launcher::AGITATOR_TYPICAL_DELAY_MICROSECONDS /
                            1'000'000.0f,
    },
    turretWidow.getTurretID());

AutoAimLaunchTimer autoAimLaunchTimer(
    aruwsrc::control::launcher::AGITATOR_TYPICAL_DELAY_MICROSECONDS,
    &drivers()->visionCoprocessor,
    &ballisticsSolver);

/* define commands ----------------------------------------------------------*/
aruwsrc::control::chassis::AutoNavCommand autoNavCommand(
    *drivers(),
    chassis,
    autoNavController,
    true);

TurretMajorSentryControlCommand majorManualCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    turretMajor,
    // turretMajorChassisYawController,
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
    getTurretMajorImu(),
    getChassisTurretMCBCanComm(),
    transformer,
    turretMajorYawLamprey,
    turretMajorYawCanEncoder,
    *turretMajorYawMotor.getEncoder(),
    turretMajor::BINNED_ALIGNMENT_OFFSET,
    turretMajor::SENTRY_YAW_ALIGNMENT_OFFSET,
    &imuCalibrateSuccessBuzzCommand,
    &imuCalibrateFailBuzzCommand);

ImuNotCalibratedGovernor imuNotCalibratedGovernor(drivers(), getTurretMajorImu());

GovernorLimitedCommand<1> imuNotCalibratedCommandLimited(
    {&buzzer},
    imuNotCalibratedCommand,
    {&imuNotCalibratedGovernor});

autotune::GravityAutotuneCommand<9, transforms::Axis::PITCH> gravityAutotuneCommandWidow(
    drivers(),
    {&turretWidow,
     &turretWidow.pitchMotor,
     &turretWidowChassisControllers.pitchController,
     turretWidowMotors.pitchMotor.isMotorInverted(),
     TURRET_WEIGHT_KG,
     DESIRED_OUT_TO_TORQUE});

autotune::LampreyAutotuneCommand<36, transforms::Axis::YAW> lampreyAutotuneCommand(
    drivers(),
    {&turretMajor,
     &turretMajor.getMutableMotor(),
     &turretMajorChassisYawController,
     turretMajorYawMotor.isMotorInverted(),
     TURRET_WEIGHT_KG,
     DESIRED_OUT_TO_TORQUE},
    turretMajorYawLamprey);

autotune::FreqSweepAutotuneCommand<transforms::Axis::YAW> freqSweepAutotuneCommand(
    drivers(),
    {&turretWidow,
     &turretWidow.yawMotor,
     &turretWidowChassisControllers.yawController,
     turretWidowMotors.yawMotor.isMotorInverted(),
     TURRET_WEIGHT_KG,
     DESIRED_OUT_TO_TORQUE},
    {.startFreq = 3.0f, .endFreq = 250.0f, .freqIncrementRatio = 1.001f, .magnitude = 12'000.0f},
    &getTurretMCBCanCommWidow(),
    {&turretWidowChassisControllers.pitchController,
     &turretMajor,
     &turretMajorChassisYawController,
     &getTurretMajorImu()});

SentryTurretCVCommand::TurretConfig turretWidowCVConfig(
    turretWidow,
    turretWidowWorldControllers.yawController,
    turretWidowWorldControllers.pitchController,
    ballisticsSolver);

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

AgitatorFanSubsystem agitatorFan(
    drivers(),
    constants::turretWidow::AGITATOR_FAN_PWM_PIN,
    constants::turretWidow::AGITATOR_FAN_PWM_TIMER,
    constants::turretWidow::AGITATOR_FAN_PWM_FREQUENCY_HZ);

AgitatorFanCommand agitatorFanCommand(
    drivers(),
    agitatorFan,
    constants::turretWidow::AGITATOR_FAN_ON_DUTY,
    constants::turretWidow::AGITATOR_FAN_OFF_DUTY,
    &imuCalibrateCommand);

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
CvOnTargetGovernor cvOnTargetGovernor(
    drivers(),
    drivers()->visionCoprocessor,
    turretCVCommand,
    autoAimLaunchTimer,
    CvOnTargetGovernorMode::ON_TARGET_AND_GATED);

// Unused, causes inconsistent fire rates due to suspected ref delay.
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
     &cvOnTargetGovernor,
     &matchRunningGovernor});

GovernorLimitedCommand<2> turretWidowAgitatorManualSpin(
    {&turretWidowAgitator},
    turretWidowRotateAndUnjamAgitator,
    {&heatLimitGovernorTurretWidow, &frictionWheelsOnGovernorTurretWidow});

aruwsrc::control::launcher::FrictionWheelLutAutotuneCommand<16>
    turretWidowLauncherLutAutotuneCommand(
        drivers(),
        {
            .frictionWheels = &turretWidowFrictionWheels,
            .manualFireCommand = &turretWidowAgitatorManualSpin,
            .barrelId = turretWidow::barrelID,
            .numFrictionWheels = 2,
            .startRpm = 4500.0f,
            .endRpm = 7500.0f,
            .rpmStep = 250.0f,
        });

/* define client display / HUD related items --------------------------------*/

// This shit is currently banned by DJI, but left for a hopeful future
ClientDisplaySubsystem clientDisplay(drivers());
tap::communication::serial::RefSerialTransmitter refSerialTransmitter(drivers());

CircleCrosshair circleCrosshair(refSerialTransmitter);
ImageIndicator imageIndicator(refSerialTransmitter);

std::vector<HudIndicator *> indicators = {&imageIndicator, &circleCrosshair};

ClientDisplayCommand clientDisplayCommand(*drivers(), clientDisplay, indicators);

/* define command mappings --------------------------------------------------*/

RemoteMapState rightUpRms = RemoteMapState({Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP});
auto rightUp = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command *>{&turretWidowFrictionWheelSpinCommand},
    &rightUpRms);

// auto nav + auto aim + cv gated fire
RemoteMapState leftUpRightUpRms = RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::UP);
auto leftUpRightUp = std::make_unique<HoldRepeatCommandMapping>(
    drivers(),
    std::vector<Command *>{&autoNavCommand, &turretCVCommand},
    &leftUpRightUpRms,
    true);

auto leftUpRightUpAg = std::make_unique<HoldRepeatCommandMapping>(
    drivers(),
    std::vector<Command *>{&turretWidowRotateAndUnjamAgitatorWithHeatAndCVLimiting},
    &leftUpRightUpRms,
    false);

// auto nav + auto aim
RemoteMapState leftUpRightMidRms =
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::MID);
auto leftUpRightMid = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command *>{&autoNavCommand, &turretCVCommand},
    &leftUpRightMidRms);

// imu calibrate
RemoteMapState leftUpRightDownRms =
    RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::DOWN);
auto leftUpRightDown = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command *>{&imuCalibrateCommand},
    &leftUpRightDownRms);

// manual aim and shoot
RemoteMapState leftMidRightUpRms =
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::UP);
auto leftMidRightUp = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command *>{&turretWidowManualCommand},
    &leftMidRightUpRms);

// manual aim and shoot
auto leftMidRightUpAg = std::make_unique<HoldRepeatCommandMapping>(
    drivers(),
    std::vector<Command *>{&turretWidowAgitatorManualSpin},
    &leftMidRightUpRms,
    false);

// auto drive & auto aim
RemoteMapState leftMidRightMidRms =
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::MID);
auto leftMidRightMid = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command *>{&majorManualCommand, &turretWidowManualCommand, &autoNavCommand},
    &leftMidRightMidRms);

// manual aim
RemoteMapState leftMidRightDownRms =
    RemoteMapState(Remote::SwitchState::MID, Remote::SwitchState::DOWN);
auto leftMidRightDown = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command *>{
        &majorManualCommand,
        &turretWidowManualCommand,
    },
    &leftMidRightDownRms);

// manual aim and shoot
RemoteMapState leftDownRightUpRms =
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::UP);
auto leftDownRightUp = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command *>{&turretWidowManualCommand},
    &leftDownRightUpRms);

auto leftDownRightUpAg = std::make_unique<HoldRepeatCommandMapping>(
    drivers(),
    std::vector<Command *>{
        &turretWidowAgitatorManualSpin,
    },
    &leftDownRightUpRms,
    false);

// manual drive & auto aim
RemoteMapState leftDownRightMidRms =
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::MID);
auto leftDownRightMid = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command *>{&chassisDriveCommand, &turretCVCommand},
    &leftDownRightMidRms);

// manual drive
RemoteMapState leftDownRightDownRms =
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::DOWN);
auto leftDownRightDown = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command *>{&chassisDriveCommand},
    &leftDownRightDownRms);

// Restart HUD
RemoteMapState ctrlBRms = RemoteMapState({Remote::Key::CTRL, Remote::Key::B});
auto bCtrlPressed = std::make_unique<PressCommandMapping>(
    drivers(),
    std::vector<Command *>{&clientDisplayCommand},
    &ctrlBRms);

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());
/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    voltageCurrentSensor.initialize();
    capBankSubsystem.initialize();
    buzzer.initialize();
    agitatorFan.initialize();
    chassis.initialize();
    turretWidow.initialize();
    turretMajor.initialize();
    turretMajorYawLamprey.initialize();
    turretMajorYawCanEncoder.initialize();
    odometrySubsystem.initialize();
    cfOdometrySubsystem.initialize();
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
    drivers->commandScheduler.registerSubsystem(&agitatorFan);
    drivers->commandScheduler.registerSubsystem(&turretMajor);
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&turretWidow);
    drivers->commandScheduler.registerSubsystem(&odometrySubsystem);
    drivers->commandScheduler.registerSubsystem(&cfOdometrySubsystem);
    drivers->commandScheduler.registerSubsystem(&transformerSubsystem);
    drivers->commandScheduler.registerSubsystem(&arucoResetSubsystem);
    drivers->commandScheduler.registerSubsystem(&clientDisplay);

    drivers->commandScheduler.registerSubsystem(&turretWidowFrictionWheels);
    drivers->commandScheduler.registerSubsystem(&turretWidowAgitator);
    drivers->commandScheduler.registerSubsystem(&capBankSubsystem);

    drivers->visionCoprocessor.attachTransformer(&transformAdapter);
    drivers->plateHitTracker.attachTransformer(&transformAdapter);
    drivers->visionCoprocessor.attachAutoNavController(&autoNavController);
    // drivers->stateMachine.attachAutoNavController(&autoNavController);
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
    agitatorFan.setDefaultCommand(&agitatorFanCommand);

    capBankSubsystem.setDefaultCommand(&sentryCapBankCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSentryCommands(Drivers *)
{
    // drivers->commandScheduler.addCommand(&imuCalibrateCommand);
    getTurretMCBCanCommWidow().setRemoteCalibrationSampleCount(4000);
    getChassisTurretMCBCanComm().setRemoteCalibrationSampleCount(4000);
    getTurretMCBCanCommWidow().setImuMountingTransforms(
        turretWidow::TURRET_MCB1_BMI088_MOUNTING_TRANSFORM,
        turretWidow::TURRET_MCB1_ISM330_MOUNTING_TRANSFORM);
    getChassisTurretMCBCanComm().setImuMountingTransforms(
        aruwsrc::control::chassis::chassisImu::CHASSIS_MCB_BMI088_MOUNTING_TRANSFORM,
        aruwsrc::control::chassis::chassisImu::CHASSIS_MCB_ISM330_MOUNTING_TRANSFORM);
}

/* register io mappings here ------------------------------------------------*/
void registerSentryIoMappings(Drivers *drivers)
{
    // commands with higher priority must be added later
    // friction wheels spin (separated due to dumb design in command mapper system)
    drivers->commandMapper.addMap(std::move(rightUp));

    drivers->commandMapper.addMap(std::move(leftDownRightMid));  // manual drive & auto aim
    drivers->commandMapper.addMap(std::move(leftDownRightUp));   // manual aim and shoot
    drivers->commandMapper.addMap(std::move(leftDownRightUpAg));
    drivers->commandMapper.addMap(std::move(leftDownRightDown));  // manual drive

    drivers->commandMapper.addMap(std::move(leftMidRightUp));  // manual aim and shoot
    drivers->commandMapper.addMap(std::move(leftMidRightUpAg));
    drivers->commandMapper.addMap(std::move(leftMidRightMid));   // auto drive & auto aim
    drivers->commandMapper.addMap(std::move(leftMidRightDown));  // manual aim

    drivers->commandMapper.addMap(std::move(leftUpRightMid));  // auto nav + auto aim
    drivers->commandMapper.addMap(std::move(leftUpRightUp));  // auto nav + auto aim + cv gated fire
    drivers->commandMapper.addMap(std::move(leftUpRightUpAg));
    drivers->commandMapper.addMap(std::move(leftUpRightDown));  // imu calibrate
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
        &sentry_control::gravityAutotuneCommandWidow,
        &sentry_control::lampreyAutotuneCommand,
        &sentry_control::freqSweepAutotuneCommand,
        &sentry_control::turretWidowLauncherLutAutotuneCommand};
    return commands;
}
#endif
// imu::ImuCalibrateCommand *getImuCalibrateCommand() { return
// &sentry_control::imuCalibrateCommand; }
