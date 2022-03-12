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
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "agitator/agitator_subsystem.hpp"
#include "agitator/constants/agitator_constants.hpp"
#include "agitator/move_unjam_ref_limited_command.hpp"
#include "aruwsrc/algorithms/odometry/otto_velocity_odometry_2d_subsystem.hpp"
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
#include "hopper-cover/open_turret_mcb_hopper_cover_command.hpp"
#include "hopper-cover/turret_mcb_hopper_cover_subsystem.hpp"
#include "imu/imu_calibrate_command.hpp"
#include "launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "turret/algorithms/chassis_frame_turret_controller.hpp"
#include "turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "turret/soldier_turret_subsystem.hpp"
#include "turret/turret_controller_constants.hpp"
#include "turret/user/turret_quick_turn_command.hpp"
#include "turret/user/turret_user_world_relative_command.hpp"

#ifdef PLATFORM_HOSTED
#include "tap/communication/can/can.hpp"
#include "tap/motor/motorsim/motor_sim.hpp"
#include "tap/motor/motorsim/sim_handler.hpp"
#endif

using namespace tap::control::setpoint;
using namespace aruwsrc::control::launcher;
using namespace aruwsrc::agitator;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::chassis;
using namespace aruwsrc::algorithms::odometry;
using namespace tap::control;
using namespace aruwsrc::control::client_display;
using namespace aruwsrc::control;
using namespace tap::communication::serial;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
aruwsrc::driversFunc drivers = aruwsrc::DoNotUse_getDrivers;

namespace soldier_control
{
/* define subsystems --------------------------------------------------------*/
tap::motor::DjiMotor pitchMotor(
    drivers(),
    SoldierTurretSubsystem::PITCH_MOTOR_ID,
    SoldierTurretSubsystem::CAN_BUS_MOTORS,
    false,
    "Pitch Turret");
tap::motor::DjiMotor yawMotor(
    drivers(),
    SoldierTurretSubsystem::YAW_MOTOR_ID,
    SoldierTurretSubsystem::CAN_BUS_MOTORS,
#ifdef TARGET_SOLDIER_2021
    false,
#else
    true,
#endif
    "Yaw Turret");
SoldierTurretSubsystem turret(drivers(), &pitchMotor, &yawMotor, false);

ChassisSubsystem chassis(drivers());

OttoVelocityOdometry2DSubsystem odometrySubsystem(drivers(), &turret, &chassis);
static inline void refreshOdom() { odometrySubsystem.refresh(); }

AgitatorSubsystem agitator(
    drivers(),
    aruwsrc::control::agitator::constants::AGITATOR_PID_CONFIG,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    aruwsrc::control::agitator::constants::AGITATOR_MOTOR_ID,
    aruwsrc::control::agitator::constants::AGITATOR_MOTOR_CAN_BUS,
    aruwsrc::control::agitator::constants::isAgitatorInverted,
    aruwsrc::control::agitator::constants::AGITATOR_JAMMING_DISTANCE,
    aruwsrc::control::agitator::constants::JAMMING_TIME,
    true);

RefereeFeedbackFrictionWheelSubsystem frictionWheels(
    drivers(),
    tap::motor::MOTOR1,
    tap::motor::MOTOR2,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

ClientDisplaySubsystem clientDisplay(drivers());

TurretMCBHopperSubsystem hopperCover(drivers());

/* define commands ----------------------------------------------------------*/
ChassisImuDriveCommand chassisImuDriveCommand(drivers(), &chassis, &turret);

ChassisDriveCommand chassisDriveCommand(drivers(), &chassis);

ChassisAutorotateCommand chassisAutorotateCommand(drivers(), &chassis, &turret, true);

BeybladeCommand beybladeCommand(drivers(), &chassis, &turret);

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

algorithms::WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController(
    drivers(),
    &turret,
    world_rel_turret_imu::PITCH_POS_PID_CONFIG,
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController(
    drivers(),
    &turret,
    world_rel_turret_imu::YAW_POS_PID_CONFIG,
    world_rel_turret_imu::YAW_VEL_PID_CONFIG);

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
    &worldFramePitchTurretImuController,
    odometrySubsystem,
    chassis,
    frictionWheels,
    1,
    1,
    14.5f);

user::TurretQuickTurnCommand turretUTurnCommand(&turret, 180.0f);

CalibrateCommand agitatorCalibrateCommand(&agitator);

MoveUnjamRefLimitedCommand agitatorShootFastLimited(
    drivers(),
    &agitator,
    M_PI / 5.0f,
    50,
    0,
    true,
    M_PI / 20.0f,
    0.4f,
    0.2f,
    140,
    2,
    true,
    10);
MoveUnjamRefLimitedCommand agitatorShootSlowLimited(
    drivers(),
    &agitator,
    M_PI / 5.0f,
    100,
    0,
    true,
    M_PI / 20.0f,
    0.4f,
    0.2f,
    140,
    2,
    true,
    10);
MoveUnjamRefLimitedCommand agitatorShootFastNotLimited(
    drivers(),
    &agitator,
    M_PI / 5.0f,
    50,
    0,
    true,
    M_PI / 20.0f,
    0.4f,
    0.2f,
    140,
    2,
    false,
    10);

FrictionWheelSpinRefLimitedCommand spinFrictionWheels(
    drivers(),
    &frictionWheels,
    15.0f,
    false,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

FrictionWheelSpinRefLimitedCommand stopFrictionWheels(
    drivers(),
    &frictionWheels,
    0.0f,
    true,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

OpenTurretMCBHopperCoverCommand openHopperCommand(&hopperCover);

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
    &hopperCover,
    frictionWheels,
    agitator,
    turret,
    imuCalibrateCommand,
    &beybladeCommand,
    &chassisAutorotateCommand,
    &chassisImuDriveCommand);

/* define command mappings --------------------------------------------------*/
// Remote related mappings
HoldCommandMapping rightSwitchDown(
    drivers(),
    {&openHopperCommand, &stopFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&agitatorShootFastLimited},
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
ToggleCommandMapping rToggled(drivers(), {&openHopperCommand}, RemoteMapState({Remote::Key::R}));
ToggleCommandMapping fToggled(drivers(), {&beybladeCommand}, RemoteMapState({Remote::Key::F}));
PressCommandMapping leftMousePressedShiftNotPressed(
    drivers(),
    {&agitatorShootSlowLimited},
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {}, {Remote::Key::SHIFT}));
HoldRepeatCommandMapping leftMousePressedShiftPressed(
    drivers(),
    {&agitatorShootFastNotLimited},
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {Remote::Key::SHIFT}),
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

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* register subsystems here -------------------------------------------------*/
void registerSoldierSubsystems(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&hopperCover);
    drivers->commandScheduler.registerSubsystem(&frictionWheels);
    drivers->commandScheduler.registerSubsystem(&clientDisplay);
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
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
    drivers->commandScheduler.addCommand(&agitatorCalibrateCommand);
    drivers->commandScheduler.addCommand(&clientDisplayCommand);
    drivers->commandScheduler.addCommand(&imuCalibrateCommand);
    drivers->visionCoprocessor.attachOdometryInterface(&odometrySubsystem);
    drivers->turretMCBCanComm.attachImuDataReceivedCallback(refreshOdom);
    drivers->visionCoprocessor.attachTurretOrientationInterface(&turret);
}

/* register io mappings here ------------------------------------------------*/
void registerSoldierIoMappings(aruwsrc::Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&rToggled);
    drivers->commandMapper.addMap(&fToggled);
    drivers->commandMapper.addMap(&leftMousePressedShiftNotPressed);
    drivers->commandMapper.addMap(&leftMousePressedShiftPressed);
    drivers->commandMapper.addMap(&rightMousePressed);
    drivers->commandMapper.addMap(&zPressed);
    drivers->commandMapper.addMap(&bNotCtrlPressedRightSwitchDown);
    drivers->commandMapper.addMap(&bCtrlPressedRightSwitchDown);
    drivers->commandMapper.addMap(&qPressed);
    drivers->commandMapper.addMap(&ePressed);
    drivers->commandMapper.addMap(&xPressed);
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

imu::ImuCalibrateCommand *getImuCalibrateCommand() { return &soldier_control::imuCalibrateCommand; }

#endif
