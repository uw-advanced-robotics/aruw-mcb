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

#include "tap/communication/sensors/encoder/can_encoder/can_encoder.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/governor/governor_with_fallback_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/algorithms/odometry/deadwheel_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/otto_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/standard_and_hero_transform_adapter.hpp"
#include "aruwsrc/algorithms/odometry/standard_and_hero_transformer.hpp"
#include "aruwsrc/algorithms/odometry/standard_and_hero_transformer_subsystem.hpp"
#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"
#include "aruwsrc/communication/can/aruw_voltage_current_sensor.hpp"
#include "aruwsrc/communication/low_battery_buzzer_command.hpp"
#include "aruwsrc/communication/serial/sentry_request_commands.hpp"
#include "aruwsrc/communication/serial/sentry_request_subsystem.hpp"
#include "aruwsrc/communication/serial/sentry_response_handler.hpp"
#include "aruwsrc/control/agitator/constant_velocity_agitator_command.hpp"
#include "aruwsrc/control/agitator/constants/agitator_constants.hpp"
#include "aruwsrc/control/agitator/manual_fire_rate_reselection_manager.hpp"
#include "aruwsrc/control/agitator/multi_shot_cv_command_mapping.hpp"
#include "aruwsrc/control/agitator/unjam_spoke_agitator_command.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/aruco/aruco_reset_subsystem.hpp"
#include "aruwsrc/control/buzzer/buzzer_subsystem.hpp"
#include "aruwsrc/control/cap_bank/cap_bank_sprint_command.hpp"
#include "aruwsrc/control/cap_bank/cap_bank_subsystem.hpp"
#include "aruwsrc/control/cap_bank/cap_bank_toggle_command.hpp"
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
#include "aruwsrc/control/client-display/indicators/vision_assistance_indicator.hpp"
#include "aruwsrc/control/client-display/indicators/matrix_hud_indicators.hpp"
#include "aruwsrc/control/client-display/indicators/text_hud_indicators.hpp"
#include "aruwsrc/control/cycle_state_command_mapping.hpp"
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
#include "aruwsrc/control/launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/cv/turret_cv_command.hpp"
#include "aruwsrc/control/turret/user/turret_quick_turn_command.hpp"
#include "aruwsrc/control/turret/user/turret_user_world_relative_command.hpp"
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
using namespace aruwsrc::agitator;
using namespace aruwsrc::algorithms;
using namespace aruwsrc::algorithms::odometry;
using namespace aruwsrc::algorithms::transforms;
using namespace aruwsrc::control;
using namespace aruwsrc::control::agitator;
using namespace aruwsrc::control::auto_aim;
using namespace aruwsrc::control::client_display;
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
inline aruwsrc::can::TurretMCBCanComm &getTurretMCBCanComm()
{
    return drivers()->turretMCBCanCommBus1;
}

/* define subsystems --------------------------------------------------------*/
tap::motor::DjiMotor pitchMotor(
    drivers(),
    PITCH_MOTOR_ID,
    CAN_BUS_MOTORS,
    true,
    "Pitch Turret",
    true,
    1,
    PITCH_MOTOR_CONFIG.startEncoderValue);

tap::motor::DjiMotor yawMotor(
    drivers(),
    YAW_MOTOR_ID,
    CAN_BUS_MOTORS,
#if defined(TARGET_STANDARD_NULL)
    false,
#else
#error "did not define standard!"
#endif
    "Yaw Turret",
    true,
    1,
    YAW_MOTOR_CONFIG.startEncoderValue);

StandardTurretSubsystem turret(
    drivers(),
    &pitchMotor,
    &yawMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG,
    &getTurretMCBCanComm());

aruwsrc::can::AruwVoltageCurrentSensor voltageCurrentSensor(drivers(), tap::can::CanBus::CAN_BUS2);

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

aruwsrc::chassis::XDriveChassisSubsystem chassis(
    drivers(),
    &voltageCurrentSensor,
    &voltageCurrentSensor,
    leftFrontChassisMotor,
    leftBackChassisMotor,
    rightFrontChassisMotor,
    rightBackChassisMotor,
    aruwsrc::chassis::WHEEL_VELOCITY_PID_CONFIG,
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

aruwsrc::algorithms::odometry::TwoDeadwheelOdometryObserver deadwheels(
    &parallelOmni,
    &perpendicularOmni,
    aruwsrc::chassis::DEADWHEEL_RADIUS);

aruwsrc::algorithms::odometry::DeadwheelKFOdometry2DSubsystem odometrySubsystem(
    *drivers(),
    deadwheels,
    turret,
    drivers()->mpu6500,
    aruwsrc::chassis::INITIAL_CHASSIS_POSITION_X,
    aruwsrc::chassis::INITIAL_CHASSIS_POSITION_Y,
    aruwsrc::chassis::CENTER_TO_WHEELBASE_RADIUS,
    aruwsrc::chassis::PARALLEL_WHEEL_CHASSIS_FORWARD_RELATIVE_ANGLE_RADIANS,
    aruwsrc::chassis::PERPENDICULAR_WHEEL_CHASSIS_FORWARD_RELATIVE_ANGLE_RADIANS);

// transforms
StandardAndHeroTransformer transformer(odometrySubsystem, turret);
StandardAnderHeroTransformerSubsystem transformSubsystem(*drivers(), transformer);

StandardAndHeroTransformAdapter transformAdapter(transformer);

VelocityAgitatorSubsystem agitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::AGITATOR_CONFIG);

aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    frictionWheels(
        drivers(),
        aruwsrc::control::launcher::LEFT_MOTOR_ID,
        aruwsrc::control::launcher::RIGHT_MOTOR_ID,
        aruwsrc::control::launcher::CAN_BUS_MOTORS,
        &getTurretMCBCanComm(),
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

OttoBallisticsSolver ballisticsSolver(
    drivers()->visionCoprocessor,
    odometrySubsystem,
    turret,
    frictionWheels,
    25.0f,  // defaultLaunchSpeed
    0       // turretID
);
AutoAimLaunchTimer autoAimLaunchTimer(
    aruwsrc::control::launcher::AGITATOR_TYPICAL_DELAY_MICROSECONDS,
    &drivers()->visionCoprocessor,
    &ballisticsSolver);

aruwsrc::control::capbank::CapBankSubsystem capBankSubsystem(drivers(), drivers()->capacitorBank);

aruwsrc::control::aruco::ArucoResetSubsystem arucoResetSubsystem(
    drivers(),
    drivers()->visionCoprocessor,
    odometrySubsystem,
    transformAdapter);

/* define commands ----------------------------------------------------------*/
aruwsrc::chassis::ChassisImuDriveCommand chassisImuDriveCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis,
    &turret.yawMotor);

aruwsrc::chassis::ChassisDriveCommand chassisDriveCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis);

aruwsrc::chassis::ChassisAutorotateCommand chassisAutorotateCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis,
    &turret.yawMotor,
    aruwsrc::chassis::ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180);

aruwsrc::chassis::WiggleDriveCommand wiggleCommand(
    drivers(),
    &chassis,
    &turret.yawMotor,
    (drivers()->controlOperatorInterface));
aruwsrc::chassis::BeybladeCommand beybladeCommand(
    drivers(),
    &chassis,
    &turret.yawMotor,
    (drivers()->controlOperatorInterface),
    aruwsrc::chassis::BEYBLADE_CONFIG);

aruwsrc::chassis::BeybladeCommand slowBeybladeCommand(
    drivers(),
    &chassis,
    &turret.yawMotor,
    (drivers()->controlOperatorInterface),
    aruwsrc::chassis::BEYBLADE_CONFIG,
    0.5f);  // Multiplier for slow beyblade speed

// Turret controllers
algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController(
    turret.pitchMotor,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
    turret.yawMotor,
    chassis_rel::YAW_PID_CONFIG);

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

algorithms::WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController(
    getTurretMCBCanComm(),
    turret.pitchMotor,
    worldFramePitchTurretImuPosPid,
    worldFramePitchTurretImuVelPid);

algorithms::WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuControllerCv(
    getTurretMCBCanComm(),
    turret.pitchMotor,
    worldFramePitchTurretImuPosPidCv,
    worldFramePitchTurretImuVelPid);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPid(world_rel_turret_imu::YAW_POS_PID_CONFIG);
tap::algorithms::SmoothPid worldFrameYawTurretImuVelPid(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController(
    getTurretMCBCanComm(),
    turret.yawMotor,
    worldFrameYawTurretImuPosPid,
    worldFrameYawTurretImuVelPid);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPidCv(
    world_rel_turret_imu::YAW_POS_PID_AUTO_AIM_CONFIG);
tap::algorithms::SmoothPid worldFrameYawTurretImuVelPidCv(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuControllerCv(
    getTurretMCBCanComm(),
    turret.yawMotor,
    worldFrameYawTurretImuPosPidCv,
    worldFrameYawTurretImuVelPidCv);

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
    &odometrySubsystem,
    {&drivers()->mpu6500});

IMUCalibrateDoneGovernor imuCalibrateDoneGovernor(drivers(), imuCalibrateCommand);

user::TurretQuickTurnCommand turretUTurnCommand(&turret, M_PI);

// beyblade governors
PlateHitGovernor plateHitGovernor(&(drivers()->plateHitTracker), 5000);

FiredRecentlyGovernor firedRecentlyGovernor(drivers(), 5000);

MovedFastRecentlyGovernor movedRecentlyGovernor(
    (drivers()->controlOperatorInterface),
    5000.0f,
    5000);

GovernorWithFallbackCommand<3> beybladeSlowWhenOutOfCombatCommand(
    {&chassis},
    slowBeybladeCommand,
    beybladeCommand,
    {&firedRecentlyGovernor, &plateHitGovernor, &movedRecentlyGovernor},
    true);
GovernorLimitedCommand<1> turretUTurnCommandLimited(
    {&turret},
    turretUTurnCommand,
    {&imuCalibrateDoneGovernor});

// base rotate/unjam commands
ConstantVelocityAgitatorCommand rotateAgitator(agitator, constants::AGITATOR_ROTATE_CONFIG);

UnjamSpokeAgitatorCommand unjamAgitator(agitator, constants::AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
    *drivers(),
    agitator,
    rotateAgitator,
    unjamAgitator);

RefSystemProjectileLaunchedGovernor refSystemProjectileLaunchedGovernor(
    drivers()->refSerial,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

FrictionWheelsOnGovernor frictionWheelsOnGovernor(frictionWheels);

ManualFireRateReselectionManager manualFireRateReselectionManager;
FireRateLimitGovernor fireRateLimitGovernor(manualFireRateReselectionManager);

GovernorLimitedCommand<3> rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched(
    {&agitator},
    rotateAndUnjamAgitator,
    {&refSystemProjectileLaunchedGovernor, &frictionWheelsOnGovernor, &fireRateLimitGovernor});

// rotates agitator with heat limiting applied
HeatLimitGovernor heatLimitGovernor(
    *drivers(),
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1,
    constants::HEAT_LIMIT_BUFFER);
GovernorLimitedCommand<1> rotateAndUnjamAgitatorWithHeatLimiting(
    {&agitator},
    rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched,
    {&heatLimitGovernor});

// rotates agitator when aiming at target and within heat limit
CvOnTargetGovernor cvOnTargetGovernor(
    ((tap::Drivers *)(drivers())),
    drivers()->visionCoprocessor,
    turretCVCommand,
    autoAimLaunchTimer,
    CvOnTargetGovernorMode::ON_TARGET_AND_GATED);

GovernorLimitedCommand<2> rotateAndUnjamAgitatorWithHeatAndCVLimiting(
    {&agitator},
    rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched,
    {&heatLimitGovernor, &cvOnTargetGovernor});

aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand spinFrictionWheels(
    drivers(),
    &frictionWheels,
    15.0f,
    false,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand stopFrictionWheels(
    drivers(),
    &frictionWheels,
    0.0f,
    true,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

aruwsrc::control::buzzer::BuzzerSubsystem buzzer(drivers());

// Cap Bank
aruwsrc::control::capbank::CapBankToggleCommand capBankToggleCommand(drivers(), capBankSubsystem);
aruwsrc::control::capbank::CapBankSprintCommand capBankSprintCommand(
    drivers(),
    capBankSubsystem,
    aruwsrc::can::capbank::SprintMode::SPRINT);
aruwsrc::control::capbank::CapBankSprintCommand capBankHalfSprintCommand(
    drivers(),
    capBankSubsystem,
    aruwsrc::can::capbank::SprintMode::HALF_SPRINT);

/* define client display / HUD related items --------------------------------*/

ClientDisplaySubsystem clientDisplay(drivers());
tap::communication::serial::RefSerialTransmitter refSerialTransmitter(drivers());

CapBankIndicator capBankIndicator(refSerialTransmitter, &drivers()->capacitorBank);

extern MultiShotCvCommandMapping leftMousePressedBNotPressed;
MatrixHudIndicators positionHudIndicators(
    *drivers(),
    drivers()->visionCoprocessor,
    refSerialTransmitter,
    frictionWheels,
    turret,
    &leftMousePressedBNotPressed,
    &cvOnTargetGovernor);

AmmoIndicator ammoIndicator(refSerialTransmitter, drivers()->refSerial);

CircleCrosshair circleCrosshair(refSerialTransmitter);

DamageIndicator damageIndicator(drivers()->plateHitTracker, turret, refSerialTransmitter);

TextHudIndicators textHudIndicators(
    *drivers(),
    agitator,
    imuCalibrateCommand,
    {&wiggleCommand, &beybladeSlowWhenOutOfCombatCommand},
    refSerialTransmitter);

VisionAssistanceIndicator visionAssistanceIndicator(
    drivers()->visionCoprocessor,
    refSerialTransmitter,
    drivers()->refSerial,
    transformAdapter.getWorldToVTM(),
    drivers()->interRobotTransmitter);

std::vector<HudIndicator *> hudIndicators = {
    &capBankIndicator,
    &positionHudIndicators,
    &ammoIndicator,
    &circleCrosshair,
    &damageIndicator,
    &textHudIndicators,
    &visionAssistanceIndicator};

ClientDisplayCommand clientDisplayCommand(*drivers(), clientDisplay, hudIndicators);

/* define command mappings --------------------------------------------------*/

// Remote related mappings
HoldRepeatCommandMapping rightSwitchMiddle(
    drivers(),
    {&spinFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID),
    true);
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&spinFrictionWheels, &rotateAndUnjamAgitatorWithHeatAndCVLimiting},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    true);

HoldRepeatCommandMapping leftSwitchDown(
    drivers(),
    {&beybladeSlowWhenOutOfCombatCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN),
    true);
HoldCommandMapping leftSwitchUp(
    drivers(),
    {&turretCVCommand, &chassisDriveCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

CycleStateCommandMapping<bool, 2, CvOnTargetGovernor> rPressed(
    drivers(),
    RemoteMapState({Remote::Key::R}),
    true,
    &cvOnTargetGovernor,
    &CvOnTargetGovernor::setGovernorEnabled);

ToggleCommandMapping fToggled(
    drivers(),
    {&beybladeSlowWhenOutOfCombatCommand},
    RemoteMapState({Remote::Key::F}));

MultiShotCvCommandMapping leftMousePressedBNotPressed(
    *drivers(),
    rotateAndUnjamAgitatorWithHeatAndCVLimiting,
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {}, {Remote::Key::B}),
    &manualFireRateReselectionManager,
    cvOnTargetGovernor,
    &rotateAgitator);

HoldRepeatCommandMapping leftMousePressedBPressed(
    drivers(),
    {&rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched},
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {Remote::Key::B}),
    false);
HoldCommandMapping rightMousePressed(
    drivers(),
    {&turretCVCommand},
    RemoteMapState(RemoteMapState::MouseButton::RIGHT));

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

// The user can press q and e simultaneously to enable wiggle driving. Wiggling is cancelled
// automatically once a different drive mode is chosen.
ToggleCommandMapping qPressed(drivers(), {&wiggleCommand}, RemoteMapState({Remote::Key::Q}));

PressCommandMapping xPressed(
    drivers(),
    {&chassisAutorotateCommand},
    RemoteMapState({Remote::Key::X}));

CycleStateCommandMapping<
    MultiShotCvCommandMapping::LaunchMode,
    MultiShotCvCommandMapping::NUM_SHOOTER_STATES,
    MultiShotCvCommandMapping>
    vPressed(
        drivers(),
        RemoteMapState({Remote::Key::V}),
        MultiShotCvCommandMapping::SINGLE,
        &leftMousePressedBNotPressed,
        &MultiShotCvCommandMapping::setShooterState,
        RemoteMapState({Remote::Key::E}));

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
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* register subsystems here -------------------------------------------------*/
void registerStandardSubsystems(Drivers *drivers)
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
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultStandardCommands(Drivers *)
{
    chassis.setDefaultCommand(&chassisAutorotateCommand);
    turret.setDefaultCommand(&turretUserWorldRelativeCommand);
    frictionWheels.setDefaultCommand(&stopFrictionWheels);
    clientDisplay.setDefaultCommand(&clientDisplayCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startStandardCommands(Drivers *drivers)
{
    // drivers->commandScheduler.addCommand(&clientDisplayCommand);
    drivers->commandScheduler.addCommand(&imuCalibrateCommand);
    drivers->visionCoprocessor.attachTransformer(&transformAdapter);
    drivers->plateHitTracker.attachTransformer(&transformAdapter);
}

/* register io mappings here ------------------------------------------------*/
void registerStandardIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchMiddle);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&rPressed);
    drivers->commandMapper.addMap(&fToggled);
    drivers->commandMapper.addMap(&leftMousePressedBNotPressed);
    drivers->commandMapper.addMap(&leftMousePressedBPressed);
    drivers->commandMapper.addMap(&rightMousePressed);
    drivers->commandMapper.addMap(&zPressed);
    drivers->commandMapper.addMap(&bNotCtrlPressedRightSwitchDown);
    drivers->commandMapper.addMap(&bCtrlPressed);
    drivers->commandMapper.addMap(&qPressed);
    drivers->commandMapper.addMap(&xPressed);
    drivers->commandMapper.addMap(&vPressed);
    drivers->commandMapper.addMap(&cShiftPressed);
    drivers->commandMapper.addMap(&shiftPressed);
    drivers->commandMapper.addMap(&ctrlPressed);
}
}  // namespace standard_control

namespace aruwsrc::standard
{
void initSubsystemCommands(aruwsrc::standard::Drivers *drivers)
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
imu::ImuCalibrateCommand *getImuCalibrateCommand()
{
    return &standard_control::imuCalibrateCommand;
}
#endif

#endif
