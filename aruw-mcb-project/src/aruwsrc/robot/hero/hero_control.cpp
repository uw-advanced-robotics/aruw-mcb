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

#if defined(TARGET_HERO_ZERO)

#include "tap/communication/sensors/encoder/can_encoder/can_encoder.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/governor/governor_with_fallback_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_absolute_command.hpp"
#include "tap/control/setpoint/commands/move_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_comprised_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/motor/double_dji_motor.hpp"

#include "aruwsrc/algorithms/odometry/chassis_cf_odometry.hpp"
#include "aruwsrc/algorithms/odometry/otto_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/transforms/standard_and_hero_transform_adapter.hpp"
#include "aruwsrc/algorithms/odometry/transforms/standard_and_hero_transformer.hpp"
#include "aruwsrc/algorithms/odometry/transforms/standard_and_hero_transformer_subsystem.hpp"
#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"
#include "aruwsrc/communication/can/aruw_voltage_current_sensor.hpp"
#include "aruwsrc/communication/low_battery_buzzer_command.hpp"
#include "aruwsrc/communication/serial/sentry_request_commands.hpp"
#include "aruwsrc/communication/serial/sentry_request_subsystem.hpp"
#include "aruwsrc/communication/serial/sentry_response_handler.hpp"
#include "aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "aruwsrc/control/agitator/constants/agitator_constants.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
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
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/wiggle_drive_command.hpp"
#include "aruwsrc/control/chassis/x_drive_chassis_subsystem.hpp"
#include "aruwsrc/control/client-display/client_display_command.hpp"
#include "aruwsrc/control/client-display/client_display_subsystem.hpp"
#include "aruwsrc/control/client-display/indicators/ammo_indicator.hpp"
#include "aruwsrc/control/client-display/indicators/cap_bank_indicator.hpp"
#include "aruwsrc/control/client-display/indicators/circle_crosshair.hpp"
#include "aruwsrc/control/client-display/indicators/damage_indicator.hpp"
#include "aruwsrc/control/client-display/indicators/enemy_indicator.hpp"
#include "aruwsrc/control/client-display/indicators/matrix_hud_indicators.hpp"
#include "aruwsrc/control/client-display/indicators/text_hud_indicators.hpp"

//#include "aruwsrc/control/client-display/indicators/vision_assistance_indicator.hpp"
#include "aruwsrc/control/client-display/old-indicators/vision_target_indicator.hpp"
#include "aruwsrc/control/cycle_state_command_mapping.hpp"
#include "aruwsrc/control/governor/cv_on_target_governor.hpp"
#include "aruwsrc/control/governor/fired_recently_governor.hpp"
#include "aruwsrc/control/governor/friction_wheels_on_governor.hpp"
#include "aruwsrc/control/governor/heat_limit_governor.hpp"
#include "aruwsrc/control/governor/imu_calibrate_done_governor.hpp"
#include "aruwsrc/control/governor/limit_switch_depressed_governor.hpp"
#include "aruwsrc/control/governor/moved_fast_recently_governor.hpp"
#include "aruwsrc/control/governor/plate_hit_governor.hpp"
#include "aruwsrc/control/governor/yellow_carded_governor.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "aruwsrc/control/launcher/friction_wheel_interface.hpp"
#include "aruwsrc/control/launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "aruwsrc/control/launcher/launcher_constants.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/cv/turret_cv_command.hpp"
#include "aruwsrc/control/turret/user/turret_quick_turn_command.hpp"
#include "aruwsrc/control/turret/user/turret_user_world_relative_command.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/hero/hero_turret_subsystem.hpp"

using namespace tap::communication::serial;
using namespace tap::control;
using namespace tap::control::governor;
using namespace tap::control::setpoint;
using namespace aruwsrc::control::agitator;
using namespace aruwsrc::algorithms;
using namespace aruwsrc::algorithms::odometry;
using namespace aruwsrc::algorithms::odometry::transforms;
using namespace aruwsrc::control::chassis;
using namespace aruwsrc::control;
using namespace aruwsrc::control::agitator;
using namespace aruwsrc::control::buzzer;
using namespace aruwsrc::control::client_display;
using namespace aruwsrc::control::client_display::indicators;
using namespace aruwsrc::control::governor;
using namespace aruwsrc::control::launcher;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::hero;
using tap::control::CommandMapper;
using tap::control::RemoteMapState;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace hero_control
{
inline aruwsrc::communication::can::TurretMCBCanComm &getTurretMCBCanComm()
{
    return drivers()->turretMCBCanCommBus1;
}

/* define subsystems --------------------------------------------------------*/
BuzzerSubsystem buzzer(drivers());

aruwsrc::communication::can::AruwVoltageCurrentSensor voltageCurrentSensor(
    drivers(),
    tap::can::CanBus::CAN_BUS2);

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

XDriveChassisSubsystem chassis(
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
std::array<tap::motor::MotorInterface *, 2> wheels = {&leftFrictionWheel, &rightFrictionWheel};
RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE,
    2>
    frictionWheelsSubsystem(
        drivers(),
        wheels,
        aruwsrc::control::launcher::WHEEL_CONFIG,
        aruwsrc::control::launcher::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT,
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM);

FrictionWheelInterface &frictionWheels = frictionWheelsSubsystem;
LaunchSpeedPredictorInterface &frictionWheelSpeedPredictor = frictionWheelsSubsystem;

VelocityAgitatorSubsystem kickerAgitator(
    drivers(),
    constants::KICKER_PID_CONFIG,
    constants::KICKER_AGITATOR_CONFIG);

VelocityAgitatorSubsystem waterwheelAgitator(
    drivers(),
    constants::WATERWHEEL_PID_CONFIG,
    constants::WATERWHEEL_AGITATOR_CONFIG);

tap::motor::DjiMotor pitchMotor(
    drivers(),
    PITCH_MOTOR_ID,
    CAN_BUS_PITCH_MOTOR,
    true,
    "Pitch Turret",
    true,
    1,
    PITCH_MOTOR_CONFIG.startEncoderValue);
tap::encoder::CanEncoder yawEncoder(
    drivers(),
    tap::encoder::CanEncoderId::ID3,
    tap::can::CanBus::CAN_BUS2,
    false,
    1.0,
    YAW_MOTOR_CONFIG.startEncoderValue);
tap::motor::DjiMotor yawMotor(
    drivers(),
    YAW_MOTOR_ID,
    CAN_BUS_YAW_MOTOR,
    false,
    "Yaw Turret",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508,
    0,
    &yawEncoder);
HeroTurretSubsystem turret(
    drivers(),
    &pitchMotor,
    &yawMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG,
    &getTurretMCBCanComm());

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
StandardAnderHeroTransformerSubsystem transformSubsystem(*drivers(), transformer);

StandardAndHeroTransformAdapter transformAdapter(transformer);

OttoBallisticsSolver ballisticsSolver(
    drivers()->visionCoprocessor,
    odometrySubsystem,
    turret,
    frictionWheelSpeedPredictor,
    15.0f,  // defaultLaunchSpeed
    0       // turretID
);
AutoAimLaunchTimer autoAimLaunchTimer(
    aruwsrc::control::launcher::AGITATOR_TYPICAL_DELAY_MICROSECONDS,
    &drivers()->visionCoprocessor,
    &ballisticsSolver);

aruwsrc::control::cap_bank::CapBankSubsystem capBankSubsystem(drivers(), drivers()->capacitorBank);

/* define commands ----------------------------------------------------------*/

ChassisImuDriveCommand chassisImuDriveCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis,
    &turret.yawMotor);

ChassisDriveCommand chassisDriveCommand(drivers(), &drivers()->controlOperatorInterface, &chassis);

ChassisAutorotateCommand chassisAutorotateCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis,
    &turret.yawMotor,
    ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_90);

BeybladeCommand beybladeCommand(
    drivers(),
    &chassis,
    &turret.yawMotor,
    (drivers()->controlOperatorInterface),
    aruwsrc::control::chassis::BEYBLADE_CONFIG);

FrictionWheelSpinRefLimitedCommand spinFrictionWheels(
    drivers(),
    &frictionWheels,
    14.0f,
    false,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM);

FrictionWheelSpinRefLimitedCommand stopFrictionWheels(
    drivers(),
    &frictionWheels,
    0.0f,
    true,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM);

// Turret Compensators
algorithms::TurretGravitationalForceOffset turretGravityCompensation(TURRET_GRAVITY_CONFIG);

// Turret controllers
algorithms::ChassisFrameTurretController<algorithms::Axis::PITCH> chassisFramePitchTurretController(
    turret.pitchMotor,
    chassis_rel::PITCH_PID_CONFIG,
    {&turretGravityCompensation});

algorithms::ChassisFrameTurretController<algorithms::Axis::YAW> chassisFrameYawTurretController(
    turret.yawMotor,
    chassis_rel::YAW_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPid(world_rel_turret_imu::YAW_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretImuVelPid(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

algorithms::WorldFrameTurretImuCascadePidTurretController<algorithms::Axis::YAW>
    worldFrameYawTurretImuController(
        transformer.getWorldToTurret(),
        getTurretMCBCanComm(),
        turret.yawMotor,
        worldFrameYawTurretImuPosPid,
        worldFrameYawTurretImuVelPid);

algorithms::WorldFrameYawChassisImuTurretController worldFrameYawChassisImuController(
    *drivers(),
    turret.yawMotor,
    world_rel_chassis_imu::YAW_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretImuPosPid(
    world_rel_turret_imu::PITCH_POS_PID_CONFIG);
tap::algorithms::SmoothPid worldFramePitchTurretImuVelPid(
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

algorithms::WorldFrameTurretImuCascadePidTurretController<algorithms::Axis::PITCH>
    worldFramePitchTurretImuController(
        transformer.getWorldToTurret(),
        getTurretMCBCanComm(),
        turret.pitchMotor,
        worldFramePitchTurretImuPosPid,
        worldFramePitchTurretImuVelPid,
        {&turretGravityCompensation});

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPidCv(
    world_rel_turret_imu::YAW_POS_PID_AUTO_AIM_CONFIG);
tap::algorithms::SmoothPid worldFrameYawTurretImuVelPidCv(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretImuPosPidCv(
    world_rel_turret_imu::PITCH_POS_PID_AUTO_AIM_CONFIG);
tap::algorithms::SmoothPid worldFramePitchTurretImuVelPidCv(
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

algorithms::WorldFrameTurretImuCascadePidTurretController<algorithms::Axis::YAW>
    worldFrameYawTurretImuControllerCv(
        transformer.getWorldToTurret(),
        getTurretMCBCanComm(),
        turret.yawMotor,
        worldFrameYawTurretImuPosPidCv,
        worldFrameYawTurretImuVelPidCv);

algorithms::WorldFrameTurretImuCascadePidTurretController<algorithms::Axis::PITCH>
    worldFramePitchTurretImuControllerCv(
        transformer.getWorldToTurret(),
        getTurretMCBCanComm(),
        turret.pitchMotor,
        worldFramePitchTurretImuPosPidCv,
        worldFramePitchTurretImuVelPidCv,
        {&turretGravityCompensation});

// turret commands
// @todo: chassis MCB is mounted vertically so world frame chassis IMU controller cannot be used for
// this
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

// beyblade governors

PlateHitGovernor plateHitGovernor(&(drivers()->plateHitTracker), 5000);

FiredRecentlyGovernor firedRecentlyGovernor(drivers(), 5000);

MovedFastRecentlyGovernor movedRecentlyGovernor(
    (drivers()->controlOperatorInterface),
    5000.0f,
    5000);

IMUCalibrateDoneGovernor imuCalibrateDoneGovernor(drivers(), imuCalibrateCommand);

user::TurretQuickTurnCommand turretUTurnCommand(&turret, M_PI);

GovernorLimitedCommand<1> turretUTurnCommandLimited(
    {&turret},
    turretUTurnCommand,
    {&imuCalibrateDoneGovernor});

// hero agitator commands

LimitSwitchDepressedGovernor limitSwitchDepressedGovernor(
    getTurretMCBCanComm(),
    LimitSwitchDepressedGovernor::LimitSwitchGovernorBehavior::READY_WHEN_DEPRESSED);
LimitSwitchDepressedGovernor limitSwitchNotDepressedGovernor(
    getTurretMCBCanComm(),
    LimitSwitchDepressedGovernor::LimitSwitchGovernorBehavior::READY_WHEN_RELEASED);

// rotates agitator if friction wheels are spinning fast
FrictionWheelsOnGovernor frictionWheelsOnGovernor(frictionWheels);

namespace waterwheel
{
MoveIntegralCommand rotateWaterwheel(
    waterwheelAgitator,
    constants::WATERWHEEL_AGITATOR_ROTATE_CONFIG);

UnjamIntegralCommand unjamWaterwheel(
    waterwheelAgitator,
    constants::WATERWHEEL_AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamWaterwheel(
    *drivers(),
    waterwheelAgitator,
    rotateWaterwheel,
    unjamWaterwheel);

GovernorLimitedCommand<2> feedWaterwheelWhenBallNotReady(
    {&waterwheelAgitator},
    rotateAndUnjamWaterwheel,
    {&limitSwitchNotDepressedGovernor, &frictionWheelsOnGovernor});
}  // namespace waterwheel

namespace kicker
{
MoveIntegralCommand loadKicker(kickerAgitator, constants::KICKER_LOAD_AGITATOR_ROTATE_CONFIG);

GovernorLimitedCommand<2> feedKickerWhenBallNotReady(
    {&kickerAgitator},
    loadKicker,
    {&limitSwitchNotDepressedGovernor, &frictionWheelsOnGovernor});

// rotates kickerAgitator when aiming at target and within heat limit
HeatLimitGovernor heatLimitGovernor(
    *drivers(),
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM,
    constants::HEAT_LIMIT_BUFFER);
CvOnTargetGovernor cvOnTargetGovernor(
    drivers(),
    drivers()->visionCoprocessor,
    turretCVCommand,
    autoAimLaunchTimer,
    CvOnTargetGovernorMode::ON_TARGET_AND_GATED);
MoveIntegralCommand launchKicker(kickerAgitator, constants::KICKER_SHOOT_AGITATOR_ROTATE_CONFIG);
GovernorLimitedCommand<1> launchKickerNoHeatLimiting(
    {&kickerAgitator},
    launchKicker,
    {&frictionWheelsOnGovernor});
GovernorLimitedCommand<3> launchKickerHeatAndCVLimited(
    {&kickerAgitator},
    launchKicker,
    {&heatLimitGovernor, &frictionWheelsOnGovernor, &cvOnTargetGovernor});
}  // namespace kicker

// @todo remove
aruwsrc::communication::serial::SentryResponseHandler sentryResponseHandler(*drivers());

// Cap Bank
aruwsrc::control::cap_bank::CapBankToggleCommand capBankToggleCommand(drivers(), capBankSubsystem);
aruwsrc::control::cap_bank::CapBankSprintCommand capBankSprintCommand(
    drivers(),
    capBankSubsystem,
    aruwsrc::communication::can::cap_bank::SprintMode::SPRINT);
aruwsrc::control::cap_bank::CapBankSprintCommand capBankHalfSprintCommand(
    drivers(),
    capBankSubsystem,
    aruwsrc::communication::can::cap_bank::SprintMode::HALF_SPRINT);

/* define client display / HUD related items --------------------------------*/

ClientDisplaySubsystem clientDisplay(drivers());
tap::communication::serial::RefSerialTransmitter refSerialTransmitter(drivers());

CapBankIndicator capBankIndicator(refSerialTransmitter, &drivers()->capacitorBank);

MatrixHudIndicators positionHudIndicators(
    *drivers(),
    drivers()->visionCoprocessor,
    refSerialTransmitter,
    frictionWheels,
    turret,
    nullptr,
    &kicker::cvOnTargetGovernor);

AmmoIndicator ammoIndicator(refSerialTransmitter, drivers()->refSerial);
EnemyIndicator enemyIndicator(refSerialTransmitter, drivers()->refSerial);
CircleCrosshair circleCrosshair(refSerialTransmitter);

DamageIndicator damageIndicator(drivers()->plateHitTracker, turret, refSerialTransmitter);

TextHudIndicators textHudIndicators(
    *drivers(),
    waterwheelAgitator,
    imuCalibrateCommand,
    {&beybladeCommand},
    refSerialTransmitter);

// VisionAssistanceIndicator visionAssistanceIndicator(
//     drivers()->visionCoprocessor,
//     refSerialTransmitter,
//     drivers()->refSerial,
//     transformAdapter.getWorldToVTM(),
//     drivers()->interRobotTransmitter);

VisionTargetIndicator visionTargetIndicator(
    drivers()->visionCoprocessor,
    refSerialTransmitter,
    transformAdapter.getWorldToVTM());

std::vector<HudIndicator *> hudIndicators = {
    &capBankIndicator,
    &positionHudIndicators,
    &ammoIndicator,
    &enemyIndicator,
    &circleCrosshair,
    &damageIndicator,
    &textHudIndicators,
    &visionTargetIndicator};

ClientDisplayCommand clientDisplayCommand(*drivers(), clientDisplay, hudIndicators);

/* define command mappings --------------------------------------------------*/
HoldCommandMapping rightSwitchMiddle(
    drivers(),
    {&spinFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&spinFrictionWheels, &kicker::launchKickerHeatAndCVLimited},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    false);
HoldCommandMapping leftSwitchDown(
    drivers(),
    {&beybladeCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
HoldCommandMapping leftSwitchUp(
    drivers(),
    {&chassisDriveCommand, &turretCVCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

MultiShotCvCommandMapping leftMousePressedBNotPressedVNotPressed(
    *drivers(),
    kicker::launchKickerHeatAndCVLimited,
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {}, {Remote::Key::B, Remote::Key::V}),
    std::nullopt,
    kicker::cvOnTargetGovernor);
HoldRepeatCommandMapping leftMousePressedBPressed(
    drivers(),
    {&kicker::launchKickerNoHeatLimiting},
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {Remote::Key::B}),
    false);
// Same thing as leftMousePressedBPressed; used for ease of access.
HoldRepeatCommandMapping leftMousePressedVPressed(
    drivers(),
    {&kicker::launchKickerNoHeatLimiting},
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {Remote::Key::V}),
    false);
HoldCommandMapping rightMousePressed(
    drivers(),
    {&turretCVCommand},
    RemoteMapState(RemoteMapState::MouseButton::RIGHT));
ToggleCommandMapping fToggled(drivers(), {&beybladeCommand}, RemoteMapState({Remote::Key::F}));
PressCommandMapping zPressed(
    drivers(),
    {&turretUTurnCommandLimited},
    RemoteMapState({Remote::Key::Z}));
// The "right switch down" portion is to avoid accidentally recalibrating in the middle of a match.
PressCommandMapping bNotCtrlPressedRightSwitchDown(
    drivers(),
    {&imuCalibrateCommand},
    RemoteMapState(
        Remote::SwitchState::UNKNOWN,
        Remote::SwitchState::DOWN,
        {Remote::Key::B},
        {Remote::Key::CTRL},
        false,
        false));
// The user can press b+ctrl when the remote right switch is in the down position to restart the
// client display command. This is necessary since we don't know when the robot is connected to the
// server and thus don't know when to start sending the initial HUD graphics.
PressCommandMapping bCtrlPressed(
    drivers(),
    {&clientDisplayCommand},
    RemoteMapState({Remote::Key::CTRL, Remote::Key::B}));

CycleStateCommandMapping<bool, 2, CvOnTargetGovernor> rPressed(
    drivers(),
    RemoteMapState({Remote::Key::R}),
    true,
    &kicker::cvOnTargetGovernor,
    &CvOnTargetGovernor::setGovernorEnabled);

// cap bank
PressCommandMapping cShiftPressed(
    drivers(),
    {&capBankToggleCommand},
    RemoteMapState({Remote::Key::SHIFT, Remote::Key::C}));
HoldCommandMapping shiftPressed(
    drivers(),
    {&capBankSprintCommand},
    RemoteMapState({Remote::Key::SHIFT}));

HoldCommandMapping ctrlPressed(
    drivers(),
    {&capBankHalfSprintCommand},
    RemoteMapState({Remote::Key::CTRL}));

// Safe disconnect function
aruwsrc::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    chassis.initialize();
    voltageCurrentSensor.initialize();
    frictionWheels.initialize();
    odometrySubsystem.initialize();
    clientDisplay.initialize();
    kickerAgitator.initialize();
    waterwheelAgitator.initialize();
    turret.initialize();
    buzzer.initialize();
    transformSubsystem.initialize();
    capBankSubsystem.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerHeroSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&frictionWheels);
    drivers->commandScheduler.registerSubsystem(&odometrySubsystem);
    drivers->commandScheduler.registerSubsystem(&clientDisplay);
    drivers->commandScheduler.registerSubsystem(&kickerAgitator);
    drivers->commandScheduler.registerSubsystem(&waterwheelAgitator);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&buzzer);
    drivers->commandScheduler.registerSubsystem(&transformSubsystem);
    drivers->commandScheduler.registerSubsystem(&capBankSubsystem);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultHeroCommands()
{
    chassis.setDefaultCommand(&chassisAutorotateCommand);
    frictionWheels.setDefaultCommand(&stopFrictionWheels);
    turret.setDefaultCommand(&turretUserWorldRelativeCommand);
    waterwheelAgitator.setDefaultCommand(&waterwheel::feedWaterwheelWhenBallNotReady);
    kickerAgitator.setDefaultCommand(&kicker::feedKickerWhenBallNotReady);
    clientDisplay.setDefaultCommand(&clientDisplayCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startHeroCommands(Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&clientDisplayCommand);
    drivers->mpu6500.setMountingTransform(
        aruwsrc::control::chassis::MPU6500_MCB_MOUNTING_TRANSFORM);
    // drivers->ism330.setMountingTransform(aruwsrc::control::chassis::ISM330_MCB_MOUNTING_TRANSFORM);
    drivers->commandScheduler.addCommand(&imuCalibrateCommand);
    drivers->visionCoprocessor.attachTransformer(&transformAdapter);
    drivers->plateHitTracker.attachTransformer(&transformAdapter);
}

/* register io mappings here ------------------------------------------------*/
void registerHeroIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchMiddle);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&leftMousePressedBNotPressedVNotPressed);
    drivers->commandMapper.addMap(&leftMousePressedBPressed);
    drivers->commandMapper.addMap(&leftMousePressedVPressed);
    drivers->commandMapper.addMap(&rightMousePressed);
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&fToggled);
    drivers->commandMapper.addMap(&zPressed);
    drivers->commandMapper.addMap(&bNotCtrlPressedRightSwitchDown);
    drivers->commandMapper.addMap(&bCtrlPressed);
    drivers->commandMapper.addMap(&rPressed);
    drivers->commandMapper.addMap(&cShiftPressed);
    drivers->commandMapper.addMap(&shiftPressed);
    drivers->commandMapper.addMap(&ctrlPressed);
}
}  // namespace hero_control

namespace aruwsrc::hero
{
void initSubsystemCommands(aruwsrc::hero::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &hero_control::remoteSafeDisconnectFunction);
    hero_control::initializeSubsystems();
    hero_control::registerHeroSubsystems(drivers);
    hero_control::setDefaultHeroCommands();
    hero_control::startHeroCommands(drivers);
    hero_control::registerHeroIoMappings(drivers);
}
}  // namespace aruwsrc::hero

#ifndef PLATFORM_HOSTED
imu::ImuCalibrateCommand *getImuCalibrateCommand() { return &hero_control::imuCalibrateCommand; }
#endif

#endif
