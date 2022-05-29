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

#ifdef ALL_SOLDIERS

#include "tap/control/command_mapper.hpp"
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/governor/governor_with_fallback_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/setpoint/commands/unjam_integral_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "agitator/constants/agitator_constants.hpp"
#include "agitator/manual_fire_rate_limiter.hpp"
#include "agitator/multi_shot_handler.hpp"
#include "agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/otto_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"
#include "aruwsrc/communication/serial/sentinel_request_commands.hpp"
#include "aruwsrc/communication/serial/sentinel_request_subsystem.hpp"
#include "aruwsrc/control/cycle_state_command_mapping.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/cv/turret_cv_command.hpp"
#include "aruwsrc/display/imu_calibrate_menu.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "chassis/beyblade_command.hpp"
#include "chassis/chassis_autorotate_command.hpp"
#include "chassis/chassis_drive_command.hpp"
#include "chassis/chassis_imu_drive_command.hpp"
#include "chassis/chassis_subsystem.hpp"
#include "client-display/client_display_command.hpp"
#include "client-display/client_display_subsystem.hpp"
#include "governor/cv_on_target_governor.hpp"
#include "governor/fire_rate_limit_governor.hpp"
#include "governor/friction_wheels_on_governor.hpp"
#include "governor/heat_limit_governor.hpp"
#include "governor/ref_system_projectile_launched_governor.hpp"
#include "hopper-cover/open_turret_mcb_hopper_cover_command.hpp"
#include "hopper-cover/turret_mcb_hopper_cover_subsystem.hpp"
#include "imu/imu_calibrate_command.hpp"
#include "launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "turret/algorithms/chassis_frame_turret_controller.hpp"
#include "turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "turret/constants/turret_constants.hpp"
#include "turret/soldier_turret_subsystem.hpp"
#include "turret/user/turret_quick_turn_command.hpp"
#include "turret/user/turret_user_world_relative_command.hpp"

#ifdef PLATFORM_HOSTED
#include "tap/communication/can/can.hpp"
#endif

using namespace tap::control::setpoint;
using namespace tap::control::governor;
using namespace aruwsrc::control::auto_aim;
using namespace aruwsrc::agitator;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::control::governor;
using namespace aruwsrc::algorithms::odometry;
using namespace aruwsrc::algorithms;
using namespace tap::control;
using namespace aruwsrc::control::client_display;
using namespace aruwsrc::control;
using namespace tap::communication::serial;
using namespace aruwsrc::control::agitator;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
aruwsrc::driversFunc drivers = aruwsrc::DoNotUse_getDrivers;

namespace soldier_control
{
inline aruwsrc::can::TurretMCBCanComm &getTurretMCBCanComm()
{
    return drivers()->turretMCBCanCommBus1;
}

/* define subsystems --------------------------------------------------------*/
aruwsrc::communication::serial::SentinelRequestSubsystem sentinelRequestSubsystem(drivers());

tap::motor::DjiMotor pitchMotor(drivers(), PITCH_MOTOR_ID, CAN_BUS_MOTORS, false, "Pitch Turret");

tap::motor::DjiMotor yawMotor(
    drivers(),
    YAW_MOTOR_ID,
    CAN_BUS_MOTORS,
#ifdef TARGET_SOLDIER_2022
    true,
#else
    false,
#endif
    "Yaw Turret");
SoldierTurretSubsystem turret(
    drivers(),
    &pitchMotor,
    &yawMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG,
    &getTurretMCBCanComm());

aruwsrc::chassis::ChassisSubsystem chassis(
    drivers(),
    aruwsrc::chassis::ChassisSubsystem::ChassisType::MECANUM);

OttoKFOdometry2DSubsystem odometrySubsystem(*drivers(), turret, chassis);

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

ClientDisplaySubsystem clientDisplay(drivers());

TurretMCBHopperSubsystem hopperCover(drivers(), getTurretMCBCanComm());

OttoBallisticsSolver ballisticsSolver(
    *drivers(),
    odometrySubsystem,
    turret,
    frictionWheels,
    14.5f,  // defaultLaunchSpeed
    0       // turretID
);
AutoAimLaunchTimer autoAimLaunchTimer(
    aruwsrc::control::launcher::AGITATOR_TYPICAL_DELAY_MICROSECONDS,
    &drivers()->visionCoprocessor,
    &ballisticsSolver);

/* define commands ----------------------------------------------------------*/
aruwsrc::communication::serial::SelectNewRobotCommand sentinelSelectNewRobotCommand(
    &sentinelRequestSubsystem);
aruwsrc::communication::serial::TargetNewQuadrantCommand sentinelTargetNewQuadrantCommand(
    &sentinelRequestSubsystem);

aruwsrc::chassis::ChassisImuDriveCommand chassisImuDriveCommand(
    drivers(),
    &chassis,
    &turret.yawMotor);

aruwsrc::chassis::ChassisDriveCommand chassisDriveCommand(drivers(), &chassis);

aruwsrc::chassis::ChassisAutorotateCommand chassisAutorotateCommand(
    drivers(),
    &chassis,
    &turret.yawMotor,
    aruwsrc::chassis::ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180);

aruwsrc::chassis::BeybladeCommand beybladeCommand(drivers(), &chassis, &turret.yawMotor);

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
tap::algorithms::SmoothPid worldFramePitchTurretImuVelPid(
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

algorithms::WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController(
    getTurretMCBCanComm(),
    turret.pitchMotor,
    worldFramePitchTurretImuPosPid,
    worldFramePitchTurretImuVelPid);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPid(world_rel_turret_imu::YAW_POS_PID_CONFIG);
tap::algorithms::SmoothPid worldFrameYawTurretImuVelPid(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController(
    getTurretMCBCanComm(),
    turret.yawMotor,
    worldFrameYawTurretImuPosPid,
    worldFrameYawTurretImuVelPid);

// turret commands
user::TurretUserWorldRelativeCommand turretUserWorldRelativeCommand(
    drivers(),
    &turret,
    &worldFrameYawChassisImuController,
    &chassisFramePitchTurretController,
    &worldFrameYawTurretImuController,
    &worldFramePitchTurretImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

cv::TurretCVCommand turretCVCommand(
    drivers(),
    &turret,
    &worldFrameYawTurretImuController,
    &worldFramePitchTurretImuController,
    &ballisticsSolver,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

user::TurretQuickTurnCommand turretUTurnCommand(&turret, M_PI);

// base rotate/unjam commands
MoveIntegralCommand rotateAgitator(agitator, constants::AGITATOR_ROTATE_CONFIG);

UnjamIntegralCommand unjamAgitator(agitator, constants::AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
    *drivers(),
    agitator,
    rotateAgitator,
    unjamAgitator);

RefSystemProjectileLaunchedGovernor refSystemProjectileLaunchedGovernor(
    drivers()->refSerial,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);
FrictionWheelsOnGovernor frictionWheelsOnGovernor(frictionWheels);
ManualFireRateLimiter manualFireRateLimiter;
FireRateLimitGovernor<ManualFireRateLimiter> fireRateLimitGovernor(
    manualFireRateLimiter,
    &ManualFireRateLimiter::getFireRatePeriod,
    &ManualFireRateLimiter::fireRateReady);
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
    *drivers(),
    turretCVCommand,
    autoAimLaunchTimer,
    CvOnTargetGovernorMode::ON_TARGET_AND_GATED);
GovernorLimitedCommand<2> rotateAndUnjamAgitatorWithHeatAndCVLimiting(
    {&agitator},
    rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched,
    {&heatLimitGovernor, &cvOnTargetGovernor});

extern HoldRepeatCommandMapping leftMousePressedBNotPressed;
MultiShotHandler multiShotHandler(leftMousePressedBNotPressed, manualFireRateLimiter, 3);

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

imu::ImuCalibrateCommand imuCalibrateCommand(
    drivers(),
    {{
        &getTurretMCBCanComm(),
        &turret,
        &chassisFrameYawTurretController,
        &chassisFramePitchTurretController,
        true,
    }},
    &chassis);

ClientDisplayCommand clientDisplayCommand(
    *drivers(),
    clientDisplay,
    &hopperCover,
    frictionWheels,
    agitator,
    turret,
    imuCalibrateCommand,
    &multiShotHandler,
    &cvOnTargetGovernor,
    &beybladeCommand,
    &chassisAutorotateCommand,
    &chassisImuDriveCommand);

/* define command mappings --------------------------------------------------*/
// Remote related mappings
HoldCommandMapping rightSwitchDown(
    drivers(),
    {&stopFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&rotateAndUnjamAgitatorWithHeatAndCVLimiting},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    true);
HoldCommandMapping leftSwitchDown(
    drivers(),
    {&beybladeCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
HoldCommandMapping leftSwitchUp(
    drivers(),
    {&turretCVCommand, &chassisDriveCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// Keyboard/Mouse related mappings
PressCommandMapping gPressedCtrlNotPressed(
    drivers(),
    {&sentinelSelectNewRobotCommand},
    RemoteMapState({Remote::Key::G}, {Remote::Key::CTRL}));
PressCommandMapping gCtrlPressed(
    drivers(),
    {&sentinelTargetNewQuadrantCommand},
    RemoteMapState({Remote::Key::G, Remote::Key::CTRL}));

CycleStateCommandMapping<bool, 2, CvOnTargetGovernor> rPressed(
    drivers(),
    RemoteMapState({Remote::Key::R}),
    true,
    &cvOnTargetGovernor,
    &CvOnTargetGovernor::setGovernorEnabled);

ToggleCommandMapping fToggled(drivers(), {&beybladeCommand}, RemoteMapState({Remote::Key::F}));
HoldRepeatCommandMapping leftMousePressedBNotPressed(
    drivers(),
    {&rotateAndUnjamAgitatorWithHeatAndCVLimiting},
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {}, {Remote::Key::B}),
    false,
    1);
HoldRepeatCommandMapping leftMousePressedBPressed(
    drivers(),
    {&rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched},
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {Remote::Key::B}),
    true);
HoldCommandMapping rightMousePressed(
    drivers(),
    {&turretCVCommand},
    RemoteMapState(RemoteMapState::MouseButton::RIGHT));
PressCommandMapping zPressed(drivers(), {&turretUTurnCommand}, RemoteMapState({Remote::Key::Z}));
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
PressCommandMapping bCtrlPressedRightSwitchDown(
    drivers(),
    {&clientDisplayCommand},
    RemoteMapState(
        Remote::SwitchState::UNKNOWN,
        Remote::SwitchState::DOWN,
        {Remote::Key::CTRL, Remote::Key::B},
        {},
        false,
        false));

PressCommandMapping qPressed(
    drivers(),
    {&chassisImuDriveCommand},
    RemoteMapState({Remote::Key::Q}));
PressCommandMapping ePressed(
    drivers(),
    {&chassisImuDriveCommand},
    RemoteMapState({Remote::Key::E}));
PressCommandMapping xPressed(
    drivers(),
    {&chassisAutorotateCommand},
    RemoteMapState({Remote::Key::X}));

CycleStateCommandMapping<
    MultiShotHandler::ShooterState,
    MultiShotHandler::NUM_SHOOTER_STATES,
    MultiShotHandler>
    vPressed(
        drivers(),
        RemoteMapState({Remote::Key::V}),
        MultiShotHandler::SINGLE,
        &multiShotHandler,
        &MultiShotHandler::setShooterState);

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* register subsystems here -------------------------------------------------*/
void registerSoldierSubsystems(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&sentinelRequestSubsystem);
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&hopperCover);
    drivers->commandScheduler.registerSubsystem(&frictionWheels);
    drivers->commandScheduler.registerSubsystem(&clientDisplay);
    drivers->commandScheduler.registerSubsystem(&odometrySubsystem);
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    sentinelRequestSubsystem.initialize();
    turret.initialize();
    chassis.initialize();
    odometrySubsystem.initialize();
    agitator.initialize();
    frictionWheels.initialize();
    hopperCover.initialize();
    clientDisplay.initialize();
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSoldierCommands(aruwsrc::Drivers *)
{
    chassis.setDefaultCommand(&chassisAutorotateCommand);
    turret.setDefaultCommand(&turretUserWorldRelativeCommand);
    frictionWheels.setDefaultCommand(&spinFrictionWheels);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSoldierCommands(aruwsrc::Drivers *drivers)
{
    // drivers->commandScheduler.addCommand(&clientDisplayCommand);
    drivers->commandScheduler.addCommand(&imuCalibrateCommand);
    drivers->visionCoprocessor.attachOdometryInterface(&odometrySubsystem);
    drivers->visionCoprocessor.attachTurretOrientationInterface(&turret, 0);
}

/* register io mappings here ------------------------------------------------*/
void registerSoldierIoMappings(aruwsrc::Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchDown);
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
    drivers->commandMapper.addMap(&bCtrlPressedRightSwitchDown);
    drivers->commandMapper.addMap(&qPressed);
    drivers->commandMapper.addMap(&ePressed);
    drivers->commandMapper.addMap(&xPressed);
    drivers->commandMapper.addMap(&gPressedCtrlNotPressed);
    drivers->commandMapper.addMap(&gCtrlPressed);
    drivers->commandMapper.addMap(&vPressed);
}
}  // namespace soldier_control

namespace aruwsrc::control
{
void initSubsystemCommands(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &soldier_control::remoteSafeDisconnectFunction);
    soldier_control::initializeSubsystems();
    soldier_control::registerSoldierSubsystems(drivers);
    soldier_control::setDefaultSoldierCommands(drivers);
    soldier_control::startSoldierCommands(drivers);
    soldier_control::registerSoldierIoMappings(drivers);
}
}  // namespace aruwsrc::control

#ifndef PLATFORM_HOSTED
imu::ImuCalibrateCommand *getImuCalibrateCommand() { return &soldier_control::imuCalibrateCommand; }
#endif

#endif
