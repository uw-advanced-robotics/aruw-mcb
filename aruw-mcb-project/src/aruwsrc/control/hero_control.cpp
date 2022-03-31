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
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_absolute_command.hpp"
#include "tap/control/setpoint/commands/move_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_comprised_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/motor/double_dji_motor.hpp"

#include "agitator/agitator_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/otto_velocity_odometry_2d_subsystem.hpp"
#include "aruwsrc/control/agitator/hero_agitator_command.hpp"
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
#include "imu/imu_calibrate_command.hpp"
#include "launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "turret/algorithms/chassis_frame_turret_controller.hpp"
#include "turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "turret/hero_turret_subsystem.hpp"
#include "turret/turret_controller_constants.hpp"
#include "turret/user/turret_quick_turn_command.hpp"
#include "turret/user/turret_user_world_relative_command.hpp"

using namespace tap::control::setpoint;
using namespace aruwsrc::chassis;
using namespace aruwsrc::control;
using namespace aruwsrc::control::turret;
using namespace tap::control;
using namespace aruwsrc::algorithms::odometry;
using namespace aruwsrc::control::client_display;
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
ChassisSubsystem chassis(drivers());

RefereeFeedbackFrictionWheelSubsystem frictionWheels(
    drivers(),
    tap::motor::MOTOR2,
    tap::motor::MOTOR1,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM,
    0.5f);

ClientDisplaySubsystem clientDisplay(drivers());

AgitatorSubsystem kickerAgitator(
    drivers(),
    aruwsrc::control::agitator::constants::PID_HERO_KICKER,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    aruwsrc::control::agitator::constants::HERO_KICKER_MOTOR_ID,
    aruwsrc::control::agitator::constants::HERO_KICKER_MOTOR_CAN_BUS,
    aruwsrc::control::agitator::constants::HERO_KICKER_INVERTED,
    0,
    0,
    false);

AgitatorSubsystem waterwheelAgitator(
    drivers(),
    aruwsrc::control::agitator::constants::PID_HERO_WATERWHEEL,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_GM3508,
    aruwsrc::control::agitator::constants::HERO_WATERWHEEL_MOTOR_ID,
    aruwsrc::control::agitator::constants::HERO_WATERWHEEL_MOTOR_CAN_BUS,
    aruwsrc::control::agitator::constants::HERO_WATERWHEEL_INVERTED,
    aruwsrc::control::agitator::constants::JAM_DISTANCE_TOLERANCE_WATERWHEEL,
    aruwsrc::control::agitator::constants::JAM_TEMPORAL_TOLERANCE_WATERWHEEL,
    true);

tap::motor::DjiMotor pitchMotor(
    drivers(),
    TurretSubsystem::PITCH_MOTOR_ID,
    TurretSubsystem::CAN_BUS_PITCH_MOTOR,
    false,
    "Pitch Turret");
tap::motor::DoubleDjiMotor yawMotor(
    drivers(),
    TurretSubsystem::YAW_BACK_MOTOR_ID,
    TurretSubsystem::YAW_FRONT_MOTOR_ID,
    TurretSubsystem::CAN_BUS_YAW_MOTORS,
    TurretSubsystem::CAN_BUS_YAW_MOTORS,
    true,
    true,
    "Yaw Front Turret",
    "Yaw Back Turret");
HeroTurretSubsystem turret(drivers(), &pitchMotor, &yawMotor, false);

OttoVelocityOdometry2DSubsystem odometrySubsystem(drivers(), &turret, &chassis);

/* define commands ----------------------------------------------------------*/
ChassisImuDriveCommand chassisImuDriveCommand(drivers(), &chassis, &turret);

ChassisDriveCommand chassisDriveCommand(drivers(), &chassis);

ChassisAutorotateCommand chassisAutorotateCommand(
    drivers(),
    &chassis,
    &turret,
    ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_90);

BeybladeCommand beybladeCommand(drivers(), &chassis, &turret);

FrictionWheelSpinRefLimitedCommand spinFrictionWheels(
    drivers(),
    &frictionWheels,
    10.0f,
    false,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_42MM);

FrictionWheelSpinRefLimitedCommand stopFrictionWheels(
    drivers(),
    &frictionWheels,
    0.0f,
    true,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_42MM);

static constexpr HeroAgitatorCommand::Config heroAgitatorCommandConfig = {
    .kickerShootRotateAngle = M_PI / 2.0,
    .kickerShootRotateTime = 75,
    .kickerShootSetpointTolerance = M_PI / 16.0f,
    .kickerLoadRotateAngle = M_PI / 2.0,
    .kickerLoadSetpointTolerance = M_PI / 16.0f,
    .waterwheelLoadRotateAngle = M_PI / 7.0,
    .waterwheelLoadSetpointTolerance = M_PI / 16.0f,
    .loadRotateTime = 200,
    .waterwheelUnjamDisplacement = M_PI / 14.0,
    .waterwheelUnjamThreshold = M_PI / 20.0,
    .waterwheelUnjamMaxWaitTime = 130,
    .heatLimiting = true,
    .heatLimitBuffer = 100,
};

HeroAgitatorCommand heroAgitatorCommand(
    drivers(),
    &kickerAgitator,
    &waterwheelAgitator,
    &frictionWheels,
    heroAgitatorCommandConfig);

// Turret controllers
algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController(
    &turret,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
    &turret,
    chassis_rel::YAW_PID_CONFIG);

algorithms::WorldFrameYawChassisImuTurretController worldFrameYawChassisImuController(
    drivers(),
    &turret,
    world_rel_chassis_imu::YAW_PID_CONFIG);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController(
    drivers(),
    &turret,
    world_rel_turret_imu::YAW_POS_PID_CONFIG,
    world_rel_turret_imu::YAW_VEL_PID_CONFIG);

algorithms::WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController(
    drivers(),
    &turret,
    world_rel_turret_imu::PITCH_POS_PID_CONFIG,
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

// turret commands
user::TurretUserWorldRelativeCommand turretUserWorldRelativeCommand(
    drivers(),
    &turret,
    &worldFrameYawChassisImuController,
    &chassisFramePitchTurretController,
    &worldFrameYawTurretImuController,
    &worldFramePitchTurretImuController);

cv::TurretCVCommand turretCVCommand(
    drivers(),
    &turret,
    &worldFrameYawTurretImuController,
    &chassisFramePitchTurretController,
    odometrySubsystem,
    chassis,
    frictionWheels,
    1,
    1,
    9.5f);

user::TurretQuickTurnCommand turretUTurnCommand(&turret, 180.0f);

imu::ImuCalibrateCommand imuCalibrateCommand(
    drivers(),
    &turret,
    &chassis,
    &chassisFrameYawTurretController,
    &chassisFramePitchTurretController,
    true);

ClientDisplayCommand clientDisplayCommand(
    drivers(),
    &clientDisplay,
    nullptr,
    frictionWheels,
    waterwheelAgitator,
    turret,
    imuCalibrateCommand,
    nullptr,
    &beybladeCommand,
    &chassisAutorotateCommand,
    &chassisImuDriveCommand);

/* define command mappings --------------------------------------------------*/
HoldCommandMapping rightSwitchDown(
    drivers(),
    {&stopFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&heroAgitatorCommand},
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
PressCommandMapping leftMousePressed(
    drivers(),
    {&heroAgitatorCommand},
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
}

/* add any starting commands to the scheduler here --------------------------*/
void startHeroCommands(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&clientDisplayCommand);
    drivers->commandScheduler.addCommand(&imuCalibrateCommand);
    drivers->visionCoprocessor.attachOdometryInterface(&odometrySubsystem);
    drivers->visionCoprocessor.attachTurretOrientationInterface(&turret);
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
