/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

/* Link for dynamic modeling & kinematics open-source: https://zhuanlan.zhihu.com/p/563048952 */

#include "aruwsrc/util_macros.hpp"

#ifdef TARGET_BALSTD

#include "tap/communication/serial/remote.hpp"
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
#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/algorithms/odometry/otto_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"
#include "aruwsrc/control/agitator/constants/agitator_constants.hpp"
#include "aruwsrc/control/agitator/manual_fire_rate_reselection_manager.hpp"
#include "aruwsrc/control/agitator/multi_shot_cv_command_mapping.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/auto-aim/auto_aim_launch_timer.hpp"
#include "aruwsrc/control/chassis/balancing/balancing_chassis_autorotate_command.hpp"
#include "aruwsrc/control/chassis/balancing/balancing_chassis_beyblade_command.hpp"
#include "aruwsrc/control/chassis/balancing/balancing_chassis_home_command.hpp"
#include "aruwsrc/control/chassis/balancing/balancing_chassis_jump_command.hpp"
#include "aruwsrc/control/chassis/balancing/balancing_chassis_recovery_command.hpp"
#include "aruwsrc/control/chassis/balancing/balancing_chassis_rel_drive_command.hpp"
#include "aruwsrc/control/chassis/balancing/balancing_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "aruwsrc/control/client-display/client_display_command.hpp"
#include "aruwsrc/control/client-display/client_display_subsystem.hpp"
#include "aruwsrc/control/cycle_state_command_mapping.hpp"
#include "aruwsrc/control/governor/fire_rate_limit_governor.hpp"
#include "aruwsrc/control/governor/friction_wheels_on_governor.hpp"
#include "aruwsrc/control/governor/heat_limit_governor.hpp"
#include "aruwsrc/control/governor/ref_system_projectile_launched_governor.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "aruwsrc/control/launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "aruwsrc/control/launcher/launcher_constants.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/motion/five_bar_motion_subsystem.hpp"
#include "aruwsrc/control/motion/five_bar_move_command.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/cv/turret_cv_command.hpp"
#include "aruwsrc/control/turret/user/turret_quick_turn_command.hpp"
#include "aruwsrc/control/turret/user/turret_user_world_relative_command.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/motor/tmotor_ak80-9.hpp"
#include "aruwsrc/robot/balstd/balstd_drivers.hpp"
#include "aruwsrc/robot/balstd/balstd_transforms.hpp"
#include "aruwsrc/robot/standard/standard_turret_subsystem.hpp"

#ifdef PLATFORM_HOSTED
#include "tap/communication/can/can.hpp"
#endif

using namespace tap::control::setpoint;
using namespace tap::control;
using namespace tap::control::governor;
using namespace aruwsrc::control::governor;
using namespace aruwsrc::control::agitator::constants;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::control::motion;
using namespace aruwsrc::chassis;
using namespace aruwsrc::balstd;
using namespace aruwsrc::agitator;
using namespace aruwsrc::control::agitator;
using namespace aruwsrc::control;
using namespace tap::communication::serial;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

inline aruwsrc::can::TurretMCBCanComm &getTurretMCBCanComm1()
{
    return drivers()->turretMCBCanCommBus1;
}
inline aruwsrc::can::TurretMCBCanComm &getTurretMCBCanComm2()
{
    return drivers()->turretMCBCanCommBus2;
}

namespace balstd_control
{
/* define subsystems --------------------------------------------------------*/
aruwsrc::motor::Tmotor_AK809 legmotorLF(
    drivers(),
    aruwsrc::motor::MOTOR3,
    tap::can::CanBus::CAN_BUS2,
    false,
    "LeftFront Leg");

aruwsrc::motor::Tmotor_AK809 legmotorLR(
    drivers(),
    aruwsrc::motor::MOTOR4,
    tap::can::CanBus::CAN_BUS2,
    false,
    "LeftRear Leg");

aruwsrc::motor::Tmotor_AK809 legmotorRF(
    drivers(),
    aruwsrc::motor::MOTOR1,
    tap::can::CanBus::CAN_BUS2,
    true,
    "RightFront Leg");

aruwsrc::motor::Tmotor_AK809 legmotorRR(
    drivers(),
    aruwsrc::motor::MOTOR2,
    tap::can::CanBus::CAN_BUS2,
    true,
    "RightRear Leg");

tap::motor::DjiMotor leftWheel(
    drivers(),
    tap::motor::MOTOR3,
    tap::can::CanBus::CAN_BUS1,
    false,
    "Left Wheel Motor");

tap::motor::DjiMotor rightWheel(
    drivers(),
    tap::motor::MOTOR4,
    tap::can::CanBus::CAN_BUS1,
    true,
    "Right Wheel Motor");

tap::motor::DjiMotor pitchMotor(
    drivers(),
    PITCH_MOTOR_ID,
    aruwsrc::control::turret::CAN_BUS_MOTORS,
    false,
    "Pitch Motor");

tap::motor::DjiMotor yawMotor(
    drivers(),
    YAW_MOTOR_ID,
    aruwsrc::control::turret::CAN_BUS_MOTORS,
    true,
    "Yaw Motor");

// END HARDWARE INIT

FiveBarLinkage fiveBarLeft(&legmotorLF, &legmotorLR, FIVE_BAR_CONFIG);

FiveBarLinkage fiveBarRight(&legmotorRF, &legmotorRR, FIVE_BAR_CONFIG);

// BEGIN SUBSYSTEMS
aruwsrc::control::turret::StandardTurretSubsystem turret(
    drivers(),
    &pitchMotor,
    &yawMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG,
    &getTurretMCBCanComm1());

aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    frictionWheels(
        drivers(),
        aruwsrc::control::launcher::LEFT_MOTOR_ID,
        aruwsrc::control::launcher::RIGHT_MOTOR_ID,
        aruwsrc::control::launcher::CAN_BUS_MOTORS,
        &getTurretMCBCanComm1(),
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

VelocityAgitatorSubsystem agitator(
    drivers(),
    aruwsrc::control::agitator::constants::AGITATOR_PID_CONFIG,
    aruwsrc::control::agitator::constants::AGITATOR_CONFIG);

aruwsrc::chassis::BalancingChassisSubsystem chassis(
    drivers(),
    getTurretMCBCanComm1(),
    getTurretMCBCanComm2(),
    turret.pitchMotor,
    turret.yawMotor,
    &fiveBarLeft,
    &fiveBarRight,
    &leftWheel,
    &rightWheel);

aruwsrc::algorithms::odometry::OttoKFOdometry2DSubsystem odometrySubsystem(
    *drivers(),
    turret,
    chassis);

client_display::ClientDisplaySubsystem clientDisplay(drivers());

// Not subsystems but not commands

aruwsrc::communication::serial::SentryResponseHandler sentryResponseHandler(*drivers());
// Ballistics Solver

aruwsrc::algorithms::OttoBallisticsSolver ballisticsSolver(
    drivers()->visionCoprocessor,
    odometrySubsystem,
    turret,
    frictionWheels,
    14.0f,  // defaultLaunchSpeed
    0       // turretID
);
AutoAimLaunchTimer autoAimLaunchTimer(
    aruwsrc::control::launcher::AGITATOR_TYPICAL_DELAY_MICROSECONDS,
    &drivers()->visionCoprocessor,
    &ballisticsSolver);

// Turret controllers
algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController(
    turret.pitchMotor,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
    turret.yawMotor,
    chassis_rel::YAW_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretImuPosPid(
    world_rel_turret_imu::PITCH_POS_PID_CONFIG);
tap::algorithms::SmoothPid worldFramePitchTurretImuPosPidCv(
    world_rel_turret_imu::PITCH_POS_PID_AUTO_AIM_CONFIG);
tap::algorithms::SmoothPid worldFramePitchTurretImuVelPid(
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG);
tap::algorithms::SmoothPid worldFramePitchTurretImuVelPidVc(
    world_rel_turret_imu::PITCH_VEL_PID_AUTO_AIM_CONFIG);

algorithms::WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController(
    getTurretMCBCanComm1(),
    turret.pitchMotor,
    worldFramePitchTurretImuPosPid,
    worldFramePitchTurretImuVelPid);

algorithms::WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuControllerCv(
    getTurretMCBCanComm1(),
    turret.pitchMotor,
    worldFramePitchTurretImuPosPidCv,
    worldFramePitchTurretImuVelPid);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPid(world_rel_turret_imu::YAW_POS_PID_CONFIG);
tap::algorithms::SmoothPid worldFrameYawTurretImuVelPid(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController(
    getTurretMCBCanComm1(),
    turret.yawMotor,
    worldFrameYawTurretImuPosPid,
    worldFrameYawTurretImuVelPid);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPidCv(
    world_rel_turret_imu::YAW_POS_PID_AUTO_AIM_CONFIG);
tap::algorithms::SmoothPid worldFrameYawTurretImuVelPidCv(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuControllerCv(
    getTurretMCBCanComm1(),
    turret.yawMotor,
    worldFrameYawTurretImuPosPidCv,
    worldFrameYawTurretImuVelPidCv);

// this goes here because it's special
cv::TurretCVCommand turretCVCommand(
    &drivers()->visionCoprocessor,
    &drivers()->controlOperatorInterface,
    &turret,
    &worldFrameYawTurretImuControllerCv,
    &worldFramePitchTurretImuControllerCv,
    &ballisticsSolver,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

// commands
// launcher commands
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

// turret commands
user::TurretUserWorldRelativeCommand turretUserWorldRelativeCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turret,
    &chassisFrameYawTurretController,
    &chassisFramePitchTurretController,
    &worldFrameYawTurretImuController,
    &worldFramePitchTurretImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

aruwsrc::control::imu::ImuCalibrateCommand imuCalibrateCommand(
    drivers(),
    {{
        &getTurretMCBCanComm1(),
        &turret,
        &chassisFrameYawTurretController,
        &chassisFramePitchTurretController,
        true,
    }},
    &chassis);

extern MultiShotCvCommandMapping leftMousePressedBNotPressed;
client_display::ClientDisplayCommand clientDisplayCommand(
    *drivers(),
    drivers()->commandScheduler,
    drivers()->visionCoprocessor,
    clientDisplay,
    nullptr,
    frictionWheels,
    agitator,
    turret,
    imuCalibrateCommand,
    &leftMousePressedBNotPressed,
    &cvOnTargetGovernor,
    nullptr,
    nullptr,
    nullptr,
    sentryResponseHandler);

aruwsrc::chassis::BalancingChassisHomeCommand homeLegCommand(drivers(), &chassis);

user::TurretQuickTurnCommand turretUTurnCommand(&turret, M_PI);

BalancingChassisRecoveryCommand recoveryCommand(
    drivers(),
    &chassis,
    drivers()->controlOperatorInterface);

BalancingChassisRelativeDriveCommand manualDriveCommand(
    drivers(),
    &chassis,
    drivers()->controlOperatorInterface);

BalancingChassisAutorotateCommand autorotateDriveCommand(
    drivers(),
    &chassis,
    drivers()->controlOperatorInterface,
    &turret.yawMotor);

BalancingChassisBeybladeCommand beybladeDriveCommand(
    drivers(),
    &chassis,
    drivers()->controlOperatorInterface,
    &turret.yawMotor);

BalancingChassisJumpCommand jumpCommand(drivers(), &chassis);

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

// Map Commands

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
    {&beybladeDriveCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
PressCommandMapping leftSwitchUp(
    drivers(),
    {&imuCalibrateCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// Keyboard/Mouse related mappings
ToggleCommandMapping gPressedCtrlNotPressed(
    drivers(),
    {&recoveryCommand},
    RemoteMapState({Remote::Key::G}, {Remote::Key::CTRL}));
PressCommandMapping gCtrlPressed(
    drivers(),
    {&homeLegCommand},
    RemoteMapState({Remote::Key::G, Remote::Key::CTRL}));

CycleStateCommandMapping<bool, 2, CvOnTargetGovernor> rPressed(
    drivers(),
    RemoteMapState({Remote::Key::R}),
    true,
    &cvOnTargetGovernor,
    &CvOnTargetGovernor::setGovernorEnabled);

ToggleCommandMapping fToggled(drivers(), {&beybladeDriveCommand}, RemoteMapState({Remote::Key::F}));

MultiShotCvCommandMapping leftMousePressedBNotPressed(
    *drivers(),
    rotateAndUnjamAgitatorWithHeatAndCVLimiting,
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {}, {Remote::Key::B}),
    &manualFireRateReselectionManager,
    cvOnTargetGovernor);
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
PressCommandMapping bCtrlPressed(
    drivers(),
    {},
    RemoteMapState({Remote::Key::CTRL, Remote::Key::B}));

PressCommandMapping qPressed(drivers(), {}, RemoteMapState({Remote::Key::Q}));
PressCommandMapping ePressed(drivers(), {&jumpCommand}, RemoteMapState({Remote::Key::E}));
PressCommandMapping xPressed(
    drivers(),
    {&autorotateDriveCommand},
    RemoteMapState({Remote::Key::X}));

CycleStateCommandMapping<
    BalancingChassisAutorotateCommand::AutorotationMode,
    BalancingChassisAutorotateCommand::NUM_AUTOROTATION_STATES,
    BalancingChassisAutorotateCommand>
    cPressed(
        drivers(),
        RemoteMapState({Remote::Key::C}),
        BalancingChassisAutorotateCommand::STRICT_PLATE_FORWARD,
        &autorotateDriveCommand,
        &BalancingChassisAutorotateCommand::setAutorotationMode);

CycleStateCommandMapping<
    MultiShotCvCommandMapping::LaunchMode,
    MultiShotCvCommandMapping::NUM_SHOOTER_STATES,
    MultiShotCvCommandMapping>
    vPressed(
        drivers(),
        RemoteMapState({Remote::Key::V}),
        MultiShotCvCommandMapping::SINGLE,
        &leftMousePressedBNotPressed,
        &MultiShotCvCommandMapping::setShooterState);

// Safe disconnect function
aruwsrc::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

// Global Objects

aruwsrc::balstd::transforms::Transformer transformer(chassis, turret);

/* register subsystems here -------------------------------------------------*/
void registerBalstdSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&frictionWheels);
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&odometrySubsystem);
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    chassis.initialize();
    turret.initialize();
    frictionWheels.initialize();
    agitator.initialize();
    odometrySubsystem.initialize();
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultBalstdCommands(Drivers *)
{
    chassis.setDefaultCommand(&autorotateDriveCommand);
    turret.setDefaultCommand(&turretUserWorldRelativeCommand);
    frictionWheels.setDefaultCommand(&spinFrictionWheels);
}

/* add any starting commands to the scheduler here --------------------------*/
void startBalstdCommands(Drivers *drivers)
{
    drivers->visionCoprocessor.attachOdometryInterface(&odometrySubsystem);
    drivers->visionCoprocessor.attachTurretOrientationInterface(&turret, 0);
}

/* register io mappings here ------------------------------------------------*/
void registerBalstdIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&bNotCtrlPressedRightSwitchDown);
    drivers->commandMapper.addMap(&rPressed);
    drivers->commandMapper.addMap(&fToggled);
    drivers->commandMapper.addMap(&leftMousePressedBNotPressed);
    drivers->commandMapper.addMap(&rightMousePressed);
    drivers->commandMapper.addMap(&zPressed);
    drivers->commandMapper.addMap(&bCtrlPressed);
    drivers->commandMapper.addMap(&qPressed);
    drivers->commandMapper.addMap(&ePressed);
    drivers->commandMapper.addMap(&xPressed);
    drivers->commandMapper.addMap(&cPressed);
    drivers->commandMapper.addMap(&gPressedCtrlNotPressed);
    drivers->commandMapper.addMap(&gCtrlPressed);
    drivers->commandMapper.addMap(&vPressed);
}
}  // namespace balstd_control

namespace aruwsrc::balstd
{
void initSubsystemCommands(Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &balstd_control::remoteSafeDisconnectFunction);
    balstd_control::initializeSubsystems();
    balstd_control::registerBalstdSubsystems(drivers);
    balstd_control::setDefaultBalstdCommands(drivers);
    balstd_control::startBalstdCommands(drivers);
    balstd_control::registerBalstdIoMappings(drivers);
}
void updateGlobals() { balstd_control::transformer.updateTransforms(); }
}  // namespace aruwsrc::balstd

#ifndef PLATFORM_HOSTED
aruwsrc::control::imu::ImuCalibrateCommand *getImuCalibrateCommand()
{
    return &balstd_control::imuCalibrateCommand;
}
#endif

#endif  // TARGET_BALSTD
