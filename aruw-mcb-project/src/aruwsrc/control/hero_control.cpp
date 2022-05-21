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

#if defined(TARGET_HERO)

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

#include "agitator/agitator_subsystem.hpp"
#include "agitator/constants/agitator_constants.hpp"
#include "agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/otto_velocity_odometry_2d_subsystem.hpp"
#include "aruwsrc/communication/serial/sentinel_request_commands.hpp"
#include "aruwsrc/communication/serial/sentinel_request_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/cv/turret_cv_command.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "chassis/beyblade_command.hpp"
#include "chassis/chassis_autorotate_command.hpp"
#include "chassis/chassis_drive_command.hpp"
#include "chassis/chassis_imu_drive_command.hpp"
#include "chassis/chassis_subsystem.hpp"
#include "chassis/wiggle_drive_command.hpp"
#include "client-display/client_display_command.hpp"
#include "client-display/client_display_subsystem.hpp"
#include "governor/cv_on_target_governor.hpp"
#include "governor/friction_wheels_on_governor.hpp"
#include "governor/heat_limit_governor.hpp"
#include "governor/limit_switch_depressed_governor.hpp"
#include "governor/yellow_carded_governor.hpp"
#include "imu/imu_calibrate_command.hpp"
#include "launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "turret/algorithms/chassis_frame_turret_controller.hpp"
#include "turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "turret/constants/turret_constants.hpp"
#include "turret/hero_turret_subsystem.hpp"
#include "turret/user/turret_quick_turn_command.hpp"
#include "turret/user/turret_user_world_relative_command.hpp"

using namespace tap::control::setpoint;
using namespace tap::control::governor;
using namespace aruwsrc::chassis;
using namespace aruwsrc::control;
using namespace aruwsrc::control::turret;
using namespace tap::control;
using namespace aruwsrc::algorithms::odometry;
using namespace aruwsrc::control::client_display;
using namespace aruwsrc::control::governor;
using namespace aruwsrc::control::agitator;
using namespace aruwsrc::agitator;
using namespace aruwsrc::control::launcher;
using namespace tap::communication::serial;
using tap::control::CommandMapper;
using tap::control::RemoteMapState;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
aruwsrc::driversFunc drivers = aruwsrc::DoNotUse_getDrivers;

namespace hero_control
{
/* define subsystems --------------------------------------------------------*/
aruwsrc::communication::serial::SentinelRequestSubsystem sentinelRequestSubsystem(drivers());

ChassisSubsystem chassis(drivers(), ChassisSubsystem::ChassisType::X_DRIVE);

RefereeFeedbackFrictionWheelSubsystem<aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    frictionWheels(
        drivers(),
        aruwsrc::control::launcher::LEFT_MOTOR_ID,
        aruwsrc::control::launcher::RIGHT_MOTOR_ID,
        aruwsrc::control::launcher::CAN_BUS_MOTORS,
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM);

ClientDisplaySubsystem clientDisplay(drivers());

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
    false,
    "Pitch Turret");
tap::motor::DoubleDjiMotor yawMotor(
    drivers(),
    YAW_BACK_MOTOR_ID,
    YAW_FRONT_MOTOR_ID,
    CAN_BUS_YAW_MOTORS,
    CAN_BUS_YAW_MOTORS,
    true,
    true,
    "Yaw Front Turret",
    "Yaw Back Turret");
HeroTurretSubsystem turret(drivers(), &pitchMotor, &yawMotor, PITCH_MOTOR_CONFIG, YAW_MOTOR_CONFIG);

OttoVelocityOdometry2DSubsystem odometrySubsystem(drivers(), &turret.yawMotor, &chassis);

/* define commands ----------------------------------------------------------*/
aruwsrc::communication::serial::SelectNewRobotCommand sentinelSelectNewRobotCommand(
    &sentinelRequestSubsystem);
aruwsrc::communication::serial::TargetNewQuadrantCommand sentinelTargetNewQuadrantCommand(
    &sentinelRequestSubsystem);

ChassisImuDriveCommand chassisImuDriveCommand(drivers(), &chassis, &turret.yawMotor);

ChassisDriveCommand chassisDriveCommand(drivers(), &chassis);

ChassisAutorotateCommand chassisAutorotateCommand(
    drivers(),
    &chassis,
    &turret.yawMotor,
    ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_90);

BeybladeCommand beybladeCommand(drivers(), &chassis, &turret.yawMotor);

FrictionWheelSpinRefLimitedCommand spinFrictionWheels(
    drivers(),
    &frictionWheels,
    10.0f,
    false,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM);

FrictionWheelSpinRefLimitedCommand stopFrictionWheels(
    drivers(),
    &frictionWheels,
    0.0f,
    true,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM);

// Turret controllers
algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController(
    &turret.pitchMotor,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
    &turret.yawMotor,
    chassis_rel::YAW_PID_CONFIG);

algorithms::WorldFrameYawChassisImuTurretController worldFrameYawChassisImuController(
    drivers(),
    &turret.yawMotor,
    world_rel_chassis_imu::YAW_PID_CONFIG);

algorithms::HeroTurretImuCascadePidTurretController worldFrameYawTurretImuController(
    drivers(),
    &turret.yawMotor,
    world_rel_turret_imu::YAW_POS_PID_CONFIG,
    world_rel_turret_imu::YAW_FUZZY_POS_PD_CONFIG,
    world_rel_turret_imu::YAW_VEL_PID_CONFIG);

algorithms::WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController(
    drivers(),
    &turret.pitchMotor,
    world_rel_turret_imu::PITCH_POS_PID_CONFIG,
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

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
    odometrySubsystem,
    frictionWheels,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR,
    9.5f);

user::TurretQuickTurnCommand turretUTurnCommand(&turret, M_PI);

imu::ImuCalibrateCommand imuCalibrateCommand(
    drivers(),
    &turret,
    &chassis,
    &chassisFrameYawTurretController,
    &chassisFramePitchTurretController,
    true);

ClientDisplayCommand clientDisplayCommand(
    *drivers(),
    clientDisplay,
    nullptr,
    frictionWheels,
    waterwheelAgitator,
    turret,
    imuCalibrateCommand,
    nullptr,
    &beybladeCommand,
    &chassisAutorotateCommand,
    &chassisImuDriveCommand);

// hero agitator commands

LimitSwitchDepressedGovernor limitSwitchDepressedGovernor(
    drivers()->turretMCBCanComm,
    LimitSwitchDepressedGovernor::LimitSwitchGovernorBehavior::READY_WHEN_DEPRESSED);
LimitSwitchDepressedGovernor limitSwitchNotDepressedGovernor(
    drivers()->turretMCBCanComm,
    LimitSwitchDepressedGovernor::LimitSwitchGovernorBehavior::READY_WHEN_RELEASED);

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

GovernorLimitedCommand<1> feedWaterwheelWhenBallNotReady(
    {&waterwheelAgitator},
    rotateAndUnjamWaterwheel,
    {&limitSwitchNotDepressedGovernor});
}  // namespace waterwheel

namespace kicker
{
MoveIntegralCommand loadKicker(kickerAgitator, constants::KICKER_LOAD_AGITATOR_ROTATE_CONFIG);

GovernorLimitedCommand<1> feedKickerWhenBallNotReady(
    {&kickerAgitator},
    loadKicker,
    {&limitSwitchNotDepressedGovernor});

MoveIntegralCommand launchKicker(kickerAgitator, constants::KICKER_SHOOT_AGITATOR_ROTATE_CONFIG);

GovernorLimitedCommand<1> launchKickerWhenBallReady(
    {&kickerAgitator},
    launchKicker,
    {&limitSwitchDepressedGovernor, &frictionWheelsOnGovernor});

// rotates agitator if friction wheels are spinning fast
FrictionWheelsOnGovernor frictionWheelsOnGovernor(frictionWheels);

// rotates kickerAgitator with heat limiting applied
HeatLimitGovernor heatLimitGovernor(
    *drivers(),
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM,
    constants::HEAT_LIMIT_BUFFER);
GovernorLimitedCommand<2> launchKickerHeatLimited(
    {&kickerAgitator},
    launchKickerWhenBallReadyWhenFrictionWheelsOn,
    {&heatLimitGovernor, &frictionWheelsOnGovernor});

// rotates kickerAgitator when aiming at target and within heat limit
CvOnTargetGovernor cvOnTargetGovernor(*drivers(), turretCVCommand);
GovernorLimitedCommand<2> launchKickerHeatAndCVLimited(
    {&kickerAgitator},
    launchKicker,
    {&heatLimitGovernor, &cvOnTargetGovernor});

// switches between normal kickerAgitator rotate and CV limited based on the yellow
// card governor
YellowCardedGovernor yellowCardedGovernor(drivers()->refSerial);
GovernorWithFallbackCommand<1> launchKickerYellowCardSwitcher(
    {&kickerAgitator},
    launchKickerHeatAndCVLimited,
    launchKickerHeatLimited,
    {&yellowCardedGovernor});
}  // namespace kicker

/* define command mappings --------------------------------------------------*/
HoldCommandMapping rightSwitchDown(
    drivers(),
    {&stopFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&kicker::launchKickerYellowCardSwitcher},
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

// Keyboard/Mouse related mappings
PressCommandMapping gPressedCtrlNotPressed(
    drivers(),
    {&sentinelSelectNewRobotCommand},
    RemoteMapState({Remote::Key::G}, {Remote::Key::CTRL}));
PressCommandMapping gCtrlPressed(
    drivers(),
    {&sentinelTargetNewQuadrantCommand},
    RemoteMapState({Remote::Key::G, Remote::Key::CTRL}));
PressCommandMapping leftMousePressed(
    drivers(),
    {&kicker::launchKickerYellowCardSwitcher},
    RemoteMapState(RemoteMapState::MouseButton::LEFT));
HoldCommandMapping rightMousePressed(
    drivers(),
    {&turretCVCommand},
    RemoteMapState(RemoteMapState::MouseButton::RIGHT));
ToggleCommandMapping fToggled(drivers(), {&beybladeCommand}, RemoteMapState({Remote::Key::F}));
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

// Safe disconnect function
aruwsrc::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    sentinelRequestSubsystem.initialize();
    chassis.initialize();
    frictionWheels.initialize();
    odometrySubsystem.initialize();
    clientDisplay.initialize();
    kickerAgitator.initialize();
    waterwheelAgitator.initialize();
    turret.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerHeroSubsystems(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&sentinelRequestSubsystem);
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&frictionWheels);
    drivers->commandScheduler.registerSubsystem(&odometrySubsystem);
    drivers->commandScheduler.registerSubsystem(&clientDisplay);
    drivers->commandScheduler.registerSubsystem(&kickerAgitator);
    drivers->commandScheduler.registerSubsystem(&waterwheelAgitator);
    drivers->commandScheduler.registerSubsystem(&turret);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultHeroCommands(aruwsrc::Drivers *)
{
    chassis.setDefaultCommand(&chassisAutorotateCommand);
    frictionWheels.setDefaultCommand(&spinFrictionWheels);
    turret.setDefaultCommand(&turretUserWorldRelativeCommand);
    waterwheelAgitator.setDefaultCommand(&waterwheel::feedWaterwheelWhenBallNotReady);
    kickerAgitator.setDefaultCommand(&kicker::feedKickerWhenBallNotReady);
}

/* add any starting commands to the scheduler here --------------------------*/
void startHeroCommands(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&clientDisplayCommand);
    drivers->commandScheduler.addCommand(&imuCalibrateCommand);
    drivers->visionCoprocessor.attachOdometryInterface(&odometrySubsystem);
    drivers->visionCoprocessor.attachTurretOrientationInterface(&turret, 0);
}

/* register io mappings here ------------------------------------------------*/
void registerHeroIoMappings(aruwsrc::Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&leftMousePressed);
    drivers->commandMapper.addMap(&rightMousePressed);
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&fToggled);
    drivers->commandMapper.addMap(&zPressed);
    drivers->commandMapper.addMap(&bNotCtrlPressedRightSwitchDown);
    drivers->commandMapper.addMap(&bCtrlPressedRightSwitchDown);
    drivers->commandMapper.addMap(&qPressed);
    drivers->commandMapper.addMap(&ePressed);
    drivers->commandMapper.addMap(&xPressed);
    drivers->commandMapper.addMap(&gPressedCtrlNotPressed);
    drivers->commandMapper.addMap(&gCtrlPressed);
}
}  // namespace hero_control

namespace aruwsrc::control
{
void initSubsystemCommands(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &hero_control::remoteSafeDisconnectFunction);
    hero_control::initializeSubsystems();
    hero_control::registerHeroSubsystems(drivers);
    hero_control::setDefaultHeroCommands(drivers);
    hero_control::startHeroCommands(drivers);
    hero_control::registerHeroIoMappings(drivers);
}
}  // namespace aruwsrc::control

imu::ImuCalibrateCommand *getImuCalibrateCommand() { return &hero_control::imuCalibrateCommand; }

#endif
