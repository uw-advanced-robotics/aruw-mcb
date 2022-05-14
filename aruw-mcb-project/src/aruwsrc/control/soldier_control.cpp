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
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/unjam_integral_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "agitator/constants/agitator_constants.hpp"
#include "agitator/multi_shot_handler.hpp"
#include "agitator/rotate_unjam_ref_limited_command.hpp"
#include "agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/otto_velocity_odometry_2d_subsystem.hpp"
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
#include "hopper-cover/open_turret_mcb_hopper_cover_command.hpp"
#include "hopper-cover/turret_mcb_hopper_cover_subsystem.hpp"
#include "imu/imu_calibrate_command.hpp"
#include "launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "ref_system/yellow_card_switcher_command.hpp"
#include "turret/algorithms/chassis_frame_turret_controller.hpp"
#include "turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "turret/constants/turret_constants.hpp"
#include "turret/cv/cv_limited_command.hpp"
#include "turret/soldier_turret_subsystem.hpp"
#include "turret/user/turret_quick_turn_command.hpp"
#include "turret/user/turret_user_world_relative_command.hpp"

#ifdef PLATFORM_HOSTED
#include "tap/communication/can/can.hpp"
#endif

using namespace tap::control::setpoint;
using namespace tap::control::setpoint;
using namespace aruwsrc::agitator;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::algorithms::odometry;
using namespace tap::control;
using namespace aruwsrc::control::client_display;
using namespace aruwsrc::control;
using namespace tap::communication::serial;
using namespace aruwsrc::control::ref_system;

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
    YAW_MOTOR_CONFIG);

aruwsrc::chassis::ChassisSubsystem chassis(
    drivers(),
    aruwsrc::chassis::ChassisSubsystem::ChassisType::MECANUM);

OttoVelocityOdometry2DSubsystem odometrySubsystem(drivers(), &turret.yawMotor, &chassis);
static inline void refreshOdom() { odometrySubsystem.refresh(); }

VelocityAgitatorSubsystem agitator(
    drivers(),
    aruwsrc::control::agitator::constants::AGITATOR_PID_CONFIG,
    aruwsrc::control::agitator::constants::AGITATOR_CONFIG);

aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem frictionWheels(
    drivers(),
    aruwsrc::control::launcher::LEFT_MOTOR_ID,
    aruwsrc::control::launcher::RIGHT_MOTOR_ID,
    aruwsrc::control::launcher::CAN_BUS_MOTORS,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1,
    0.1f);

ClientDisplaySubsystem clientDisplay(drivers());

TurretMCBHopperSubsystem hopperCover(drivers(), drivers()->turretMCBCanCommBus1);

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
    &turret.pitchMotor,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
    &turret.yawMotor,
    chassis_rel::YAW_PID_CONFIG);

algorithms::WorldFrameYawChassisImuTurretController worldFrameYawChassisImuController(
    drivers(),
    &turret.yawMotor,
    world_rel_chassis_imu::YAW_PID_CONFIG);

algorithms::WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController(
    drivers()->turretMCBCanCommBus1,
    &turret.pitchMotor,
    world_rel_turret_imu::PITCH_POS_PID_CONFIG,
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController(
    drivers()->turretMCBCanCommBus1,
    &turret.yawMotor,
    world_rel_turret_imu::YAW_POS_PID_CONFIG,
    world_rel_turret_imu::YAW_VEL_PID_CONFIG);

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
    14.5f);

user::TurretQuickTurnCommand turretUTurnCommand(&turret, M_PI);

MoveIntegralCommand agitatorRotateCommand(
    agitator,
    aruwsrc::control::agitator::constants::AGITATOR_ROTATE_CONFIG);

UnjamIntegralCommand agitatorUnjamCommand(
    agitator,
    aruwsrc::control::agitator::constants::AGITATOR_UNJAM_CONFIG);

RotateUnjamRefLimitedCommand agitatorShootFastLimited(
    *drivers(),
    agitator,
    agitatorRotateCommand,
    agitatorUnjamCommand,
    aruwsrc::control::agitator::constants::HEAT_LIMIT_BUFFER);

MoveUnjamIntegralComprisedCommand agitatorShootFastUnlimited(
    *drivers(),
    agitator,
    agitatorRotateCommand,
    agitatorUnjamCommand);

extern HoldRepeatCommandMapping leftMousePressedShiftNotPressed;
MultiShotHandler multiShotHandler(&leftMousePressedShiftNotPressed, 3);

aruwsrc::control::turret::cv::CVLimitedCommand agitatorLaunchCVLimited(
    *drivers(),
    {&agitator},
    agitatorShootFastLimited,
    turretCVCommand);

YellowCardSwitcherCommand agitatorLaunchYellowCardCommand(
    *drivers(),
    {&agitator},
    agitatorShootFastLimited,
    agitatorLaunchCVLimited);

aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand spinFrictionWheels(
    drivers(),
    &frictionWheels,
    15.0f,
    false,
    aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand stopFrictionWheels(
    drivers(),
    &frictionWheels,
    0.0f,
    true,
    aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM_1);

OpenTurretMCBHopperCoverCommand openHopperCommand(&hopperCover);

imu::ImuCalibrateCommand imuCalibrateCommand(
    drivers(),
    {
        std::tuple<
            aruwsrc::can::TurretMCBCanComm *,
            aruwsrc::control::turret::TurretSubsystem *,
            aruwsrc::control::turret::algorithms::ChassisFrameYawTurretController *,
            aruwsrc::control::turret::algorithms::ChassisFramePitchTurretController *>(
            &drivers()->turretMCBCanCommBus1,
            &turret,
            &chassisFrameYawTurretController,
            &chassisFramePitchTurretController),
    },
    &chassis,
    true);

ClientDisplayCommand clientDisplayCommand(
    *drivers(),
    clientDisplay,
    &hopperCover,
    frictionWheels,
    agitator,
    turret,
    imuCalibrateCommand,
    &multiShotHandler,
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
    {&agitatorLaunchYellowCardCommand},
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

ToggleCommandMapping rToggled(drivers(), {&openHopperCommand}, RemoteMapState({Remote::Key::R}));
ToggleCommandMapping fToggled(drivers(), {&beybladeCommand}, RemoteMapState({Remote::Key::F}));
HoldRepeatCommandMapping leftMousePressedShiftNotPressed(
    drivers(),
    {&agitatorLaunchYellowCardCommand},
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {}, {Remote::Key::SHIFT}),
    false,
    1);
HoldRepeatCommandMapping leftMousePressedShiftPressed(
    drivers(),
    {&agitatorShootFastUnlimited},
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
    drivers->turretMCBCanCommBus1.attachImuDataReceivedCallback(refreshOdom);
    drivers->visionCoprocessor.attachTurretOrientationInterface(&turret, 0);
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

imu::ImuCalibrateCommand *getImuCalibrateCommand() { return &soldier_control::imuCalibrateCommand; }

#endif
