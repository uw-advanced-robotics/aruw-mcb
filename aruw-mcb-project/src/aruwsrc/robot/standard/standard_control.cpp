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

#ifdef ALL_STANDARDS

#include <memory>

#include "tap/communication/sensors/encoder/can_encoder/can_encoder.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/command_composition_helper.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/governor/governor_with_fallback_command.hpp"
#include "tap/control/instant_command.hpp"
#include "tap/control/remote_map_state.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/timeout_command.hpp"
#include "tap/control/trigger.hpp"
#include "tap/control/trigger_helpers.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/algorithms/ballistics/cv_ballistics_solver.hpp"
#include "aruwsrc/algorithms/odometry/chassis_cf_odometry.hpp"
#include "aruwsrc/algorithms/odometry/otto_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/three_deadwheel_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/transforms/standard_and_hero_transform_adapter.hpp"
#include "aruwsrc/algorithms/odometry/transforms/standard_and_hero_transformer.hpp"
#include "aruwsrc/algorithms/odometry/transforms/standard_and_hero_transformer_subsystem.hpp"
#include "aruwsrc/communication/can/aruw_voltage_current_sensor.hpp"
#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/communication/low_battery_buzzer_command.hpp"
#include "aruwsrc/control/agitator/constant_fire_rate_agitator_command.hpp"
#include "aruwsrc/control/agitator/constants/agitator_constants.hpp"
#include "aruwsrc/control/agitator/manual_fire_rate_reselection_manager.hpp"
#include "aruwsrc/control/agitator/multi_shot_cv_command.hpp"
#include "aruwsrc/control/agitator/unjam_spoke_agitator_command.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/aruco/aruco_reset_subsystem.hpp"
#include "aruwsrc/control/autotune/freq_sweep_autotune.hpp"
#include "aruwsrc/control/autotune/gravity_autotune.hpp"
#include "aruwsrc/control/autotune/second_order_autotune.hpp"
#include "aruwsrc/control/autotune/spring_autotune.hpp"
#include "aruwsrc/control/buzzer/buzzer_subsystem.hpp"
#include "aruwsrc/control/buzzer/note_sequence_command.hpp"
#include "aruwsrc/control/buzzer/note_sequences.hpp"
#include "aruwsrc/control/cap-bank/cap_bank_sprint_command.hpp"
#include "aruwsrc/control/cap-bank/cap_bank_subsystem.hpp"
#include "aruwsrc/control/cap-bank/cap_bank_toggle_command.hpp"
#include "aruwsrc/control/chassis/beyblade_command.hpp"
#include "aruwsrc/control/chassis/chassis_autorotate_command.hpp"
#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "aruwsrc/control/chassis/chassis_imu_drive_command.hpp"
#include "aruwsrc/control/chassis/wiggle_drive_command.hpp"
#include "aruwsrc/control/chassis/x_drive_chassis_subsystem.hpp"
#include "aruwsrc/control/client-display/client_display_command.hpp"
#include "aruwsrc/control/client-display/client_display_subsystem.hpp"
#include "aruwsrc/control/client-display/indicators/ammo_indicator.hpp"
#include "aruwsrc/control/client-display/indicators/cap_bank_indicator.hpp"
#include "aruwsrc/control/client-display/indicators/circle_crosshair.hpp"
#include "aruwsrc/control/client-display/indicators/damage_indicator.hpp"
#include "aruwsrc/control/client-display/indicators/matrix_hud_indicators.hpp"
#include "aruwsrc/control/client-display/indicators/text_hud_indicators.hpp"
#include "aruwsrc/control/client-display/old-indicators/vision_target_indicator.hpp"
#include "aruwsrc/control/cycle_state_mode_controller.hpp"
#include "aruwsrc/control/governor/cv_on_target_governor.hpp"
#include "aruwsrc/control/governor/fire_rate_limit_governor.hpp"
#include "aruwsrc/control/governor/fired_recently_governor.hpp"
#include "aruwsrc/control/governor/friction_wheels_on_governor.hpp"
#include "aruwsrc/control/governor/heat_limit_governor.hpp"
#include "aruwsrc/control/governor/imu_calibrate_done_governor.hpp"
#include "aruwsrc/control/governor/moved_fast_recently_governor.hpp"
#include "aruwsrc/control/governor/plate_hit_governor.hpp"
#include "aruwsrc/control/governor/ref_system_projectile_launched_governor.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "aruwsrc/control/launcher/friction_wheel_lut_autotune_command.hpp"
#include "aruwsrc/control/launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "aruwsrc/control/launcher/launcher_constants.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/third_order_compensation.hpp"
#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"
#include "aruwsrc/control/turret/algorithms/turret_spring_compensation.hpp"
#include "aruwsrc/control/turret/algorithms/turret_stos_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_stos_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/cv/turret_cv_command.hpp"
#include "aruwsrc/control/turret/user/turret_quick_turn_command.hpp"
#include "aruwsrc/control/turret/user/turret_user_world_relative_command.hpp"
#include "aruwsrc/display/autotune_menu.hpp"
#include "aruwsrc/display/imu_calibrate_menu.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/standard/standard_chassis_constants.hpp"
#include "aruwsrc/robot/standard/standard_drivers.hpp"
#include "aruwsrc/robot/standard/standard_turret_subsystem.hpp"

#ifdef PLATFORM_HOSTED
#include "tap/communication/can/can.hpp"
#endif

using namespace tap::communication::serial;
using namespace tap::control;

using namespace tap::control::setpoint;
using namespace tap::control::governor;
using namespace aruwsrc::algorithms::odometry;
using namespace aruwsrc::control::agitator;
using namespace aruwsrc::algorithms;
using namespace aruwsrc::algorithms::ballistics;
using namespace aruwsrc::algorithms::odometry;
using namespace aruwsrc::algorithms::odometry::transforms;
using namespace aruwsrc::control;
using namespace aruwsrc::control::agitator;
using namespace aruwsrc::control::auto_aim;
using namespace aruwsrc::control::buzzer;
using namespace aruwsrc::control::client_display::indicators;
using namespace aruwsrc::control::governor;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::standard;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;
namespace standard_control
{
inline aruwsrc::communication::can::TurretMCBCanComm& getTurretMCBCanComm()
{
    return drivers()->turretMCBCanCommBus1;
}
using Compose = CommandCompositionHelper;

/* define subsystems ----------------------------c---------------------------*/
BuzzerSubsystem buzzer(drivers());

tap::motor::DjiMotor pitchMotor(
    drivers(),
    PITCH_MOTOR_ID,
    CAN_BUS_MOTORS,
    true,
    "Pitch Turret",
    true,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_GM6020,
    PITCH_MOTOR_CONFIG.startEncoderValue);

tap::motor::DjiMotor yawMotor(
    drivers(),
    YAW_MOTOR_ID,
    CAN_BUS_MOTORS,
    false,
    "Yaw Turret",
    true,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_GM6020,
    YAW_MOTOR_CONFIG.startEncoderValue);

aruwsrc::control::turret::TurretMotor pitchTurretMotor(&pitchMotor, PITCH_MOTOR_CONFIG);
aruwsrc::control::turret::TurretMotor yawTurretMotor(&yawMotor, YAW_MOTOR_CONFIG);

StandardTurretSubsystem turret(drivers(), pitchTurretMotor, yawTurretMotor, &getTurretMCBCanComm());

aruwsrc::communication::can::AruwVoltageCurrentSensor voltageCurrentSensor(
    drivers(),
    tap::can::CanBus::CAN_BUS2);

tap::motor::DjiMotor leftFrontChassisMotor(
    drivers(),
    aruwsrc::control::chassis::LEFT_FRONT_MOTOR_ID,
    aruwsrc::control::chassis::CAN_BUS_MOTORS,
    aruwsrc::control::chassis::WHEELBASE_MOTOR_INVERTED,
    "Left Front Chassis Motor",
    false,
    aruwsrc::control::chassis::CHASSIS_GEARBOX_RATIO);

tap::motor::DjiMotor leftBackChassisMotor(
    drivers(),
    aruwsrc::control::chassis::LEFT_BACK_MOTOR_ID,
    aruwsrc::control::chassis::CAN_BUS_MOTORS,
    aruwsrc::control::chassis::WHEELBASE_MOTOR_INVERTED,
    "Left Back Chassis Motor",
    false,
    aruwsrc::control::chassis::CHASSIS_GEARBOX_RATIO);

tap::motor::DjiMotor rightFrontChassisMotor(
    drivers(),
    aruwsrc::control::chassis::RIGHT_FRONT_MOTOR_ID,
    aruwsrc::control::chassis::CAN_BUS_MOTORS,
    aruwsrc::control::chassis::WHEELBASE_MOTOR_INVERTED,
    "Right Front Chassis Motor",
    false,
    aruwsrc::control::chassis::CHASSIS_GEARBOX_RATIO);

tap::motor::DjiMotor rightBackChassisMotor(
    drivers(),
    aruwsrc::control::chassis::RIGHT_BACK_MOTOR_ID,
    aruwsrc::control::chassis::CAN_BUS_MOTORS,
    aruwsrc::control::chassis::WHEELBASE_MOTOR_INVERTED,
    "Right Back Chassis Motor",
    false,
    aruwsrc::control::chassis::CHASSIS_GEARBOX_RATIO);

aruwsrc::control::chassis::XDriveChassisSubsystem chassis(
    drivers(),
    &voltageCurrentSensor,
    &voltageCurrentSensor,
    leftFrontChassisMotor,
    leftBackChassisMotor,
    rightFrontChassisMotor,
    rightBackChassisMotor,
    aruwsrc::control::chassis::WHEEL_VELOCITY_PID_CONFIG,
    aruwsrc::control::chassis::WHEEL_RADIUS,
    aruwsrc::control::chassis::WHEELBASE_RADIUS,
    &drivers()->capacitorBank);

tap::encoder::CanEncoder parallelOmni(
    drivers(),
    tap::encoder::CanEncoderId::ID1,
    tap::can::CanBus::CAN_BUS2,
    true);

tap::encoder::CanEncoder perpendicularOmni(
    drivers(),
    tap::encoder::CanEncoderId::ID0,
    tap::can::CanBus::CAN_BUS2);

aruwsrc::algorithms::odometry::OttoChassisWorldYawObserver yawObserver(turret);
aruwsrc::algorithms::odometry::ChassisCFOdometry odometrySubsystem(
    drivers(),
    chassis,
    yawObserver,
    // drivers()->ism330,
    drivers()->mpu6500,
    modm::Vector2f(
        aruwsrc::control::chassis::INITIAL_CHASSIS_POSITION_X,
        aruwsrc::control::chassis::INITIAL_CHASSIS_POSITION_Y));

// transforms
StandardAndHeroTransformer transformer(odometrySubsystem, turret);
StandardAnderHeroTransformerSubsystem transformSubsystem(
    *drivers(),
    transformer,
    &drivers()->rttTelemetry);

StandardAndHeroTransformAdapter transformAdapter(transformer);

VelocityAgitatorSubsystem agitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::AGITATOR_CONFIG);

tap::motor::DjiMotor leftFrictionWheel(
    drivers(),
    aruwsrc::control::launcher::LEFT_MOTOR_ID,
    aruwsrc::control::launcher::CAN_BUS_MOTORS,
    true,
    "Left flywheel");
tap::motor::DjiMotor rightFrictionWheel(
    drivers(),
    aruwsrc::control::launcher::RIGHT_MOTOR_ID,
    aruwsrc::control::launcher::CAN_BUS_MOTORS,
    false,
    "Right flywheel");
std::array<tap::motor::MotorInterface*, 2> wheels = {&leftFrictionWheel, &rightFrictionWheel};

aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE,
    2>
    frictionWheels(
        drivers(),
        wheels,
        aruwsrc::control::launcher::WHEEL_CONFIG,
        aruwsrc::control::launcher::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT,
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM,
        aruwsrc::control::launcher::LAUNCHER_SPEED_CORRECTION_PID_CONFIG);

CvBallisticsSolver ballisticsSolver(
    drivers()->visionCoprocessor,
    transformAdapter,
    frictionWheels,
    {
        .shotTimingEntryThreshold = SHOT_TIMING_ENTRY_THRESHOLD,
        .shotTimingExitThreshold = SHOT_TIMING_EXIT_THRESHOLD,
        .defaultLaunchSpeed = aruwsrc::control::launcher::LAUNCHER_SPEED,
        .turretPitchOffset = 0,
        .minimumShotDelay = aruwsrc::control::launcher::AGITATOR_TYPICAL_DELAY_MICROSECONDS /
                            1'000'000.0f,
    },
    0,  // turretID
    &drivers()->rttTelemetry);

AutoAimLaunchTimer autoAimLaunchTimer(
    aruwsrc::control::launcher::AGITATOR_TYPICAL_DELAY_MICROSECONDS,
    &drivers()->visionCoprocessor,
    &ballisticsSolver);

aruwsrc::control::cap_bank::CapBankSubsystem capBankSubsystem(
    drivers(),
    drivers()->capacitorBank,
    voltageCurrentSensor);

aruwsrc::control::aruco::ArucoResetSubsystem arucoResetSubsystem(
    drivers(),
    drivers()->visionCoprocessor,
    odometrySubsystem,
    transformAdapter);

tap::control::Subsystem dummySubsystem(drivers());

/* define commands ----------------------------------------------------------*/
aruwsrc::control::chassis::ChassisImuDriveCommand chassisImuDriveCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis,
    &turret.yawMotor);

aruwsrc::control::chassis::ChassisDriveCommand chassisDriveCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis);

aruwsrc::control::chassis::ChassisAutorotateCommand chassisAutorotateCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis,
    &turret.yawMotor,
    aruwsrc::control::chassis::ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180);

aruwsrc::control::chassis::WiggleDriveCommand wiggleCommand(
    drivers(),
    &chassis,
    &turret.yawMotor,
    (drivers()->controlOperatorInterface));

aruwsrc::control::chassis::BeybladeCommand beybladeCommand(
    drivers(),
    &chassis,
    &turret.yawMotor,
    (drivers()->controlOperatorInterface),
    aruwsrc::control::chassis::BEYBLADE_CONFIG);

// Turret compensators

algorithms::TurretGravitationalForceOffset turretGravityCompensation(TURRET_GRAVITY_CONFIG);

algorithms::TurretSpringForceOffset turretSpringCompensation(
    TURRET_SPRING_CONFIG,
    pitchMotor.isMotorInverted());

// Turret controllers
algorithms::ChassisFrameTurretController<tap::algorithms::transforms::Axis::PITCH>
    chassisFramePitchTurretController(
        turret.pitchMotor,
        chassis_rel::PITCH_PID_CONFIG,
        {&turretGravityCompensation, &turretSpringCompensation});

algorithms::ChassisFrameTurretController<tap::algorithms::transforms::Axis::YAW>
    chassisFrameYawTurretController(turret.yawMotor, chassis_rel::YAW_PID_CONFIG);

algorithms::WorldFrameYawChassisImuTurretController worldFrameYawChassisImuController(
    *drivers(),
    turret.yawMotor,
    world_rel_chassis_imu::YAW_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretImuPosPid(
    world_rel_turret_imu::PITCH_POS_PID_CONFIG);
tap::algorithms::SmoothPid worldFramePitchTurretImuPosPidCv(
    world_rel_turret_imu::PITCH_POS_PID_AUTO_AIM_CONFIG);
tap::algorithms::SmoothPid worldFramePitchTurretImuVelPid(
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

algorithms::WorldFrameTurretImuCascadePidTurretController<tap::algorithms::transforms::Axis::PITCH>
    worldFramePitchTurretImuController(
        transformer.getWorldToTurret(),
        getTurretMCBCanComm(),
        turret.pitchMotor,
        worldFramePitchTurretImuPosPid,
        worldFramePitchTurretImuVelPid,
        {&turretGravityCompensation, &turretSpringCompensation});

algorithms::WorldFrameTurretImuCascadePidTurretController<tap::algorithms::transforms::Axis::PITCH>
    worldFramePitchTurretImuControllerCv(
        transformer.getWorldToTurret(),
        getTurretMCBCanComm(),
        turret.pitchMotor,
        worldFramePitchTurretImuPosPidCv,
        worldFramePitchTurretImuVelPid,
        {&turretGravityCompensation, &turretSpringCompensation});

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPid(world_rel_turret_imu::YAW_POS_PID_CONFIG);
tap::algorithms::SmoothPid worldFrameYawTurretImuVelPid(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

algorithms::WorldFrameTurretImuCascadePidTurretController<tap::algorithms::transforms::Axis::YAW>
    worldFrameYawTurretImuController(
        transformer.getWorldToTurret(),
        getTurretMCBCanComm(),
        turret.yawMotor,
        worldFrameYawTurretImuPosPid,
        worldFrameYawTurretImuVelPid);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPidCv(
    world_rel_turret_imu::YAW_POS_PID_AUTO_AIM_CONFIG);
tap::algorithms::SmoothPid worldFrameYawTurretImuVelPidCv(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

#if defined(TARGET_STANDARD_NULL)
algorithms::WorldFrameTurretImuCascadePidTurretController<tap::algorithms::transforms::Axis::YAW>
    worldFrameYawTurretImuControllerCv(
        transformer.getWorldToTurret(),
        getTurretMCBCanComm(),
        turret.yawMotor,
        worldFrameYawTurretImuPosPidCv,
        worldFrameYawTurretImuVelPidCv);
#else
algorithms::WorldFrameTurretImuSTOSTurretController<tap::algorithms::transforms::Axis::YAW>
    worldFrameYawTurretImuControllerCv(
        transformer.getWorldToTurret(),
        getTurretMCBCanComm(),
        turret.yawMotor,
        world_rel_turret_imu::STOS_CONSTANTS,
        worldFrameYawTurretImuPosPidCv,
        world_rel_turret_imu::FEEDFORWARD_CONSTANTS);
#endif

// turret commands
user::TurretUserWorldRelativeCommand turretUserWorldRelativeCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turret,
    &worldFrameYawChassisImuController,
    &chassisFramePitchTurretController,
    &worldFrameYawTurretImuController,
    &worldFramePitchTurretImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

cv::TurretCVCommand turretCVCommand(
    &drivers()->visionCoprocessor,
    &drivers()->controlOperatorInterface,
    &turret,
    &worldFrameYawTurretImuControllerCv,
    &worldFramePitchTurretImuControllerCv,
    &ballisticsSolver,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

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

autotune::GravityAutotuneCommand<9, tap::algorithms::transforms::Axis::PITCH> gravityAutotuneCommand(
    drivers(),
    {&turret,
     &turret.pitchMotor,
     &chassisFramePitchTurretController,
     pitchMotor.isMotorInverted(),
     TURRET_WEIGHT_KG,
     TORQUE_TO_DESIRED_OUT},
    &turretSpringCompensation,
    &chassis,
    {},
    modm::toRadian(0.003));

autotune::SpringAutotuneCommand<9, tap::algorithms::transforms::Axis::PITCH> springAutotuneCommand(
    drivers(),
    {&turret,
     &turret.pitchMotor,
     &chassisFramePitchTurretController,
     pitchMotor.isMotorInverted(),
     TURRET_WEIGHT_KG,
     TORQUE_TO_DESIRED_OUT},
    &turretSpringCompensation,
    &turretGravityCompensation,
    &chassis,
    {},
    &imuCalibrateSuccessBuzzCommand,
    &imuCalibrateFailBuzzCommand,
    modm::toRadian(0.003));

autotune::SecondOrderAutotuneCommand<9, tap::algorithms::transforms::Axis::PITCH>
    secondOrderAutotuneCommand(
        drivers(),
        {&turret,
         &turret.pitchMotor,
         &chassisFramePitchTurretController,
         pitchMotor.isMotorInverted(),
         TURRET_WEIGHT_KG,
         TORQUE_TO_DESIRED_OUT},
        &turretGravityCompensation,
        &chassis,
        {},
        &imuCalibrateSuccessBuzzCommand,
        &imuCalibrateFailBuzzCommand);

autotune::FreqSweepAutotuneCommand<tap::algorithms::transforms::Axis::YAW> freqSweepAutotuneCommand(
    drivers(),
    {&turret,
     &turret.yawMotor,
     &chassisFrameYawTurretController,
     yawMotor.isMotorInverted(),
     TURRET_WEIGHT_KG,
     TORQUE_TO_DESIRED_OUT},
    {.startFreq = 1.5f, .endFreq = 250.0f, .freqIncrementRatio = 1.0001f, .magnitude = 12'000.0f},
    &getTurretMCBCanComm(),
    {&chassisFramePitchTurretController});

user::TurretQuickTurnCommand turretUTurnCommand(&turret, M_PI);

// beyblade governors
PlateHitGovernor plateHitGovernor(&(drivers()->plateHitTracker), 5000);

FiredRecentlyGovernor firedRecentlyGovernor(drivers(), 5000);

MovedFastRecentlyGovernor movedRecentlyGovernor(
    (drivers()->controlOperatorInterface),
    5000.0f,
    5000);

GovernorLimitedCommand<1> turretUTurnCommandLimited(
    {&turret},
    turretUTurnCommand,
    {&imuCalibrateDoneGovernor});

// base rotate/unjam commands
ManualFireRateReselectionManager manualFireRateReselectionManager;

ConstantFireRateAgitatorCommand rotateAgitator(
    agitator,
    ConstantFireRateAgitatorCommand::Config{
        constants::AGITATOR_ROTATE_CONFIG,
        constants::MANUAL_CONSTANT_FIRE_RATE_RPS,
        constants::AGITATOR_NUM_POCKETS,
        constants::MIN_CONSTANT_FIRE_RATE_RPM,
        &manualFireRateReselectionManager});
MoveIntegralCommand rotateAgitatorSingleShot(agitator, constants::AGITATOR_ROTATE_CONFIG);

UnjamSpokeAgitatorCommand unjamAgitator(agitator, constants::AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
    *drivers(),
    agitator,
    rotateAgitatorSingleShot,
    unjamAgitator);

// Unused, causes incosnistent fire rates due to suspected ref delay.
// RefSystemProjectileLaunchedGovernor refSystemProjectileLaunchedGovernor(
//     drivers()->refSerial,
//     tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM);

FrictionWheelsOnGovernor frictionWheelsOnGovernor(frictionWheels);

GovernorLimitedCommand<1> rotateAndUnjamAgitatorWhenFrictionWheelsOn(
    {&agitator},
    rotateAndUnjamAgitator,
    {&frictionWheelsOnGovernor});

// rotates agitator with heat limiting applied
HeatLimitGovernor heatLimitGovernor(
    *drivers(),
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM,
    constants::HEAT_LIMIT_BUFFER);
GovernorLimitedCommand<1> rotateAndUnjamAgitatorWithHeatLimiting(
    {&agitator},
    rotateAndUnjamAgitatorWhenFrictionWheelsOn,
    {&heatLimitGovernor});

GovernorLimitedCommand<2> agitatorManualSpin(
    {&agitator},
    rotateAndUnjamAgitator,
    {&heatLimitGovernor, &frictionWheelsOnGovernor});

aruwsrc::control::launcher::FrictionWheelLutAutotuneCommand<24> launcherLutAutotuneCommand(
    drivers(),
    {
        .frictionWheels = &frictionWheels,
        .manualFireCommand = &agitatorManualSpin,
        .barrelId = tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM,
        .numFrictionWheels = 2,
        .startRpm = 3000.0f,
        .endRpm = 8000.0f,
        .rpmStep = 250.0f,
    });

// rotates agitator when aiming at target and within heat limit
CvOnTargetGovernor cvOnTargetGovernor(
    ((tap::Drivers*)(drivers())),
    drivers()->visionCoprocessor,
    turretCVCommand,
    autoAimLaunchTimer,
    CvOnTargetGovernorMode::ON_TARGET_AND_GATED);

GovernorLimitedCommand<2> rotateAndUnjamAgitatorWithHeatAndCVLimiting(
    {&agitator},
    rotateAndUnjamAgitatorWhenFrictionWheelsOn,
    {&heatLimitGovernor, &cvOnTargetGovernor});

// GovernorLimitedCommand<3> rotateAndUnjamAgitatorWithHeatAndCVWindowLimiting(
//     {&agitator},
//     rotateAndUnjamAgitator,
//     {&frictionWheelsOnGovernor, &cvOnTargetGovernor, &heatLimitGovernor});

aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand spinFrictionWheels(
    drivers(),
    &frictionWheels,
    15.0f,
    false,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM);

aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand stopFrictionWheels(
    drivers(),
    &frictionWheels,
    0.0f,
    true,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM);

// Cap Bank
aruwsrc::control::cap_bank::CapBankToggleCommand capBankToggleCommand(drivers(), capBankSubsystem);
aruwsrc::control::cap_bank::CapBankSprintCommand capBankSprintCommand(
    drivers(),
    capBankSubsystem,
    aruwsrc::communication::can::cap_bank::SprintMode::SPRINT);

/* define client display / HUD related items --------------------------------*/

aruwsrc::control::client_display::ClientDisplaySubsystem clientDisplay(drivers());
tap::communication::serial::RefSerialTransmitter refSerialTransmitter(drivers());

CapBankIndicator capBankIndicator(refSerialTransmitter, &drivers()->capacitorBank);

AmmoIndicator ammoIndicator(refSerialTransmitter, drivers()->refSerial);

CircleCrosshair circleCrosshair(refSerialTransmitter);

DamageIndicator damageIndicator(drivers()->plateHitTracker, turret, refSerialTransmitter);

TextHudIndicators textHudIndicators(
    *drivers(),
    agitator,
    imuCalibrateCommand,
    {&wiggleCommand, &beybladeCommand},
    refSerialTransmitter);

// VisionAssistanceIndicator visionAssistanceIndicator(
//     drivers()->visionCoprocessor,
//     refSerialTransmitter,
//     drivers()->refSerial,
//     transformAdapter.getWorldToVTM(),
//     drivers()->interRobotTransmitter);

VisionTargetIndicator visionTargetIndicator(
    drivers()->visionCoprocessor,
    ballisticsSolver,
    refSerialTransmitter,
    transformAdapter.getWorldToVTM());

std::vector<HudIndicator*> hudIndicators = {
    &capBankIndicator,
    &textHudIndicators,
    &ammoIndicator,
    &circleCrosshair,
    // &damageIndicator,
    &textHudIndicators,
    &visionTargetIndicator};

aruwsrc::control::client_display::ClientDisplayCommand clientDisplayCommand(
    *drivers(),
    clientDisplay,
    hudIndicators);

// Remote related mappings
Trigger rightSwitchMiddle =
    TriggerHelpers::switchState(drivers(), Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID)
        .onTrue(&spinFrictionWheels)
        .onFalse(&stopFrictionWheels);

Trigger rightSwitchUp =
    TriggerHelpers::switchState(drivers(), Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP)
        .onTrue(&spinFrictionWheels)
        .whileTrue(&rotateAndUnjamAgitatorWithHeatAndCVLimiting);

Trigger leftSwitchDown =
    TriggerHelpers::switchState(drivers(), Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN)
        .whileTrue(&beybladeCommand);

Trigger leftSwitchUp =
    TriggerHelpers::switchState(drivers(), Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP)
        .whileTrue(&turretCVCommand)  // shouldn't be composed into a concurrent command as
                                      // cvOnTargetGoverner checks if this command specifically is
                                      // scheduled
        .whileTrue(&chassisDriveCommand);

Trigger fToggled = TriggerHelpers::button(drivers(), Remote::Key::F).toggleOnTrue(&beybladeCommand);

Trigger leftMousePressedBPressed = (TriggerHelpers::leftMouseButton(drivers()) &&
                                    TriggerHelpers::button(drivers(), Remote::Key::B))
                                       .whileTrue(&rotateAndUnjamAgitatorWhenFrictionWheelsOn);

Trigger rightMousePressed = TriggerHelpers::rightMouseButton(drivers()).whileTrue(&turretCVCommand);

Trigger zPressed =
    TriggerHelpers::button(drivers(), Remote::Key::Z).onTrue(&turretUTurnCommandLimited);

// The "right switch down" portion is to avoid accidentally recalibrating in the middle of a match.
Trigger bNotCtrlPressedRightSwitchDown =
    (TriggerHelpers::switchState(
         drivers(),
         Remote::Switch::RIGHT_SWITCH,
         Remote::SwitchState::DOWN) &&
     TriggerHelpers::button(drivers(), Remote::Key::B) &&
     !TriggerHelpers::button(drivers(), Remote::Key::CTRL) &&
     !TriggerHelpers::leftMouseButton(drivers()) && !TriggerHelpers::rightMouseButton(drivers()))
        .onTrue(&imuCalibrateCommand);

// The user can press b+ctrl when the remote right switch is in the down position to restart the
// client display command. This is necessary since we don't know when the robot is connected to the
// server and thus don't know when to start sending the initial HUD graphics.
Trigger bCtrlPressed = (TriggerHelpers::button(drivers(), Remote::Key::B) &&
                        TriggerHelpers::button(drivers(), Remote::Key::CTRL))
                           .onTrue(&clientDisplayCommand);

// The user can press q to enable wiggle driving. Wiggling is cancelled
// automatically once a different drive mode is chosen.
Trigger qPressed = TriggerHelpers::button(drivers(), Remote::Key::Q).toggleOnTrue(&wiggleCommand);

Trigger xPressed =
    TriggerHelpers::button(drivers(), Remote::Key::X).onTrue(&chassisAutorotateCommand);

MultiShotCvCommand multiShotCvCommand(
    *drivers(),
    rotateAndUnjamAgitatorWithHeatAndCVLimiting,
    &manualFireRateReselectionManager,
    cvOnTargetGovernor,
    &rotateAgitator);

MatrixHudIndicators positionHudIndicators(
    *drivers(),
    drivers()->visionCoprocessor,
    refSerialTransmitter,
    frictionWheels,
    turret,
    &multiShotCvCommand,
    &cvOnTargetGovernor);

// Compose::parallel<1> so that multishot is still a weakconcurrentcommand and isReady is bypassed
// since trigger doesn't have ownership
Trigger leftMousePressed =
    TriggerHelpers::leftMouseButton(drivers()).whileTrue(&multiShotCvCommand);
//.whileTrue(Compose::parallel<2>({&turretCVCommand, &multiShotCvCommand}));

auto cycleStateController = CycleStateModeController<
    MultiShotCvCommand::LaunchMode,
    MultiShotCvCommand::NUM_SHOOTER_STATES,
    MultiShotCvCommand>(
    MultiShotCvCommand::LIMITED_20HZ,
    &multiShotCvCommand,
    &MultiShotCvCommand::setShooterState);

InstantCommand incrementCycleShootCommand(
    []() { cycleStateController.cycleState(); },
    std::array<tap::control::Subsystem*, 1>{
        &dummySubsystem});  // fake requirement so gets scheduled by command scheduler

Trigger vPressed =
    TriggerHelpers::button(drivers(), Remote::Key::V).onTrue(&incrementCycleShootCommand);

InstantCommand decrementCycleShootCommand(
    []() { cycleStateController.reverseCycleState(); },
    std::array<tap::control::Subsystem*, 1>{&dummySubsystem});

Trigger ePressed =
    TriggerHelpers::button(drivers(), Remote::Key::E).onTrue(&decrementCycleShootCommand);

auto cycleStateGovernor = CycleStateModeController<bool, 2, CvOnTargetGovernor>(
    true,
    &cvOnTargetGovernor,
    &CvOnTargetGovernor::setGovernorEnabled);

InstantCommand toggleGovernorMode(
    []() { cycleStateGovernor.cycleState(); },
    std::array<tap::control::Subsystem*, 1>{&dummySubsystem});

Trigger rPressed = TriggerHelpers::button(drivers(), Remote::Key::R).onTrue(&toggleGovernorMode);

// cap bank
Trigger cShiftPressed = (TriggerHelpers::button(drivers(), Remote::Key::C) &&
                         TriggerHelpers::button(drivers(), Remote::Key::SHIFT))
                            .onTrue(&capBankToggleCommand);

Trigger shiftPressed =
    TriggerHelpers::button(drivers(), Remote::Key::SHIFT).whileTrue(&capBankSprintCommand);

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* register subsystems here -------------------------------------------------*/
void registerStandardSubsystems(Drivers* drivers)
{
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&frictionWheels);
    drivers->commandScheduler.registerSubsystem(&clientDisplay);
    drivers->commandScheduler.registerSubsystem(&odometrySubsystem);
    drivers->commandScheduler.registerSubsystem(&buzzer);
    drivers->commandScheduler.registerSubsystem(&transformSubsystem);
    drivers->commandScheduler.registerSubsystem(&capBankSubsystem);
    drivers->commandScheduler.registerSubsystem(&arucoResetSubsystem);
    drivers->commandScheduler.registerSubsystem(&dummySubsystem);
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    turret.initialize();
    voltageCurrentSensor.initialize();
    chassis.initialize();
    odometrySubsystem.initialize();
    agitator.initialize();
    frictionWheels.initialize();
    clientDisplay.initialize();
    buzzer.initialize();
    transformSubsystem.initialize();
    capBankSubsystem.initialize();
    arucoResetSubsystem.initialize();
    perpendicularOmni.initialize();
    parallelOmni.initialize();
    dummySubsystem.initialize();
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultStandardCommands(Drivers*)
{
    chassis.setDefaultCommand(&chassisAutorotateCommand);
    turret.setDefaultCommand(&turretUserWorldRelativeCommand);
    frictionWheels.setDefaultCommand(&stopFrictionWheels);
    clientDisplay.setDefaultCommand(&clientDisplayCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startStandardCommands(Drivers* drivers)
{
    // drivers->commandScheduler.addCommand(&clientDisplayCommand);
    drivers->commandScheduler.addCommand(&imuCalibrateCommand);
    drivers->visionCoprocessor.attachTransformer(&transformAdapter);
    drivers->plateHitTracker.attachTransformer(&transformAdapter);
#if defined(TARGET_STANDARD_PHOBOS) || defined(TARGET_STANDARD_DEIMOS)
    getTurretMCBCanComm().setImuMountingTransforms(
        aruwsrc::control::turret::TURRET_MCB_BMI088_MOUNTING_TRANSFORM,
        aruwsrc::control::turret::TURRET_MCB_ISM330_MOUNTING_TRANSFORM);
#endif
#ifdef TARGET_STANDARD_NULL
    getTurretMCBCanComm().setImuMountingTransform(
        aruwsrc::communication::can::TurretMCBCanComm::RemoteImuType::BMI088,
        aruwsrc::control::turret::TURRET_MCB_BMI088_MOUNTING_TRANSFORM);
#endif
    // drivers->ism330.setMountingTransform(
    //     tap::algorithms::transforms::Transform(0.02578, 0.09607, 0, 0, 0, 0));
}

/* register io mappings here ------------------------------------------------*/
void registerStandardIoMappings(Drivers*) {}
}  // namespace standard_control

namespace aruwsrc::standard
{
void initSubsystemCommands(aruwsrc::standard::Drivers* drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &standard_control::remoteSafeDisconnectFunction);
    standard_control::initializeSubsystems();
    standard_control::registerStandardSubsystems(drivers);
    standard_control::setDefaultStandardCommands(drivers);
    standard_control::startStandardCommands(drivers);
    standard_control::registerStandardIoMappings(drivers);
}
}  // namespace aruwsrc::standard

#ifndef PLATFORM_HOSTED
imu::ImuCalibrateCommand* getImuCalibrateCommand()
{
    return &standard_control::imuCalibrateCommand;
}

std::vector<aruwsrc::control::autotune::TurretAutotuneInterface*> getAutotuneCommands()
{
    static std::vector<aruwsrc::control::autotune::TurretAutotuneInterface*> commands = {
        &standard_control::gravityAutotuneCommand,
        &standard_control::springAutotuneCommand,
        &standard_control::freqSweepAutotuneCommand,
        &standard_control::secondOrderAutotuneCommand,
        &standard_control::launcherLutAutotuneCommand,
    };
    return commands;
}
#endif

#endif
