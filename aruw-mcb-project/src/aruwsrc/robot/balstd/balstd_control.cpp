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

#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"
#include "aruwsrc/control/agitator/constants/agitator_constants.hpp"
#include "aruwsrc/control/agitator/manual_fire_rate_reselection_manager.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/auto-aim/auto_aim_launch_timer.hpp"
#include "aruwsrc/control/chassis/balancing/balancing_chassis_autorotate_command.hpp"
#include "aruwsrc/control/chassis/balancing/balancing_chassis_beyblade_command.hpp"
#include "aruwsrc/control/chassis/balancing/balancing_chassis_home_command.hpp"
#include "aruwsrc/control/chassis/balancing/balancing_chassis_rel_drive_command.hpp"
#include "aruwsrc/control/chassis/balancing/balancing_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
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
#include "aruwsrc/control/turret/cv/turret_cv_command.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
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
using namespace tap::communication::serial;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

inline aruwsrc::can::TurretMCBCanComm &getTurretMCBCanComm()
{
    return drivers()->turretMCBCanCommBus1;
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
    tap::motor::MOTOR4,
    tap::can::CanBus::CAN_BUS1,
    false,
    "Left Wheel Motor");

tap::motor::DjiMotor rightWheel(
    drivers(),
    tap::motor::MOTOR3,
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

BalancingLeg legLeft(
    drivers(),
    getTurretMCBCanComm(),
    &fiveBarLeft,
    LF_LEG_MOTOR_PID_CONFIG,
    LF_LEG_MOTOR_FUZZY_PID_CONFIG,
    LR_LEG_MOTOR_PID_CONFIG,
    LR_LEG_MOTOR_FUZZY_PID_CONFIG,
    &leftWheel,
    LEFT_WHEEL_MOTOR_PID_CONFIG);

BalancingLeg legRight(
    drivers(),
    getTurretMCBCanComm(),
    &fiveBarRight,
    RF_LEG_MOTOR_PID_CONFIG,
    RF_LEG_MOTOR_FUZZY_PID_CONFIG,
    RR_LEG_MOTOR_PID_CONFIG,
    RR_LEG_MOTOR_FUZZY_PID_CONFIG,
    &rightWheel,
    RIGHT_WHEEL_MOTOR_PID_CONFIG);

// BEGIN SUBSYSTEMS
aruwsrc::control::turret::StandardTurretSubsystem turret(
    drivers(),
    &pitchMotor,
    &yawMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG,
    &getTurretMCBCanComm());

aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    frictionWheels(
        drivers(),
        aruwsrc::control::launcher::LEFT_MOTOR_ID,
        aruwsrc::control::launcher::RIGHT_MOTOR_ID,
        aruwsrc::control::launcher::CAN_BUS_MOTORS,
        &getTurretMCBCanComm(),
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

VelocityAgitatorSubsystem agitator(
    drivers(),
    aruwsrc::control::agitator::constants::AGITATOR_PID_CONFIG,
    aruwsrc::control::agitator::constants::AGITATOR_CONFIG);

aruwsrc::chassis::BalancingChassisSubsystem chassis(
    drivers(),
    getTurretMCBCanComm(),
    turret.pitchMotor,
    turret.yawMotor,
    legLeft,
    legRight);

aruwsrc::algorithms::odometry::OttoKFOdometry2DSubsystem odometryTracker(
    drivers(),
    &turret,
    &chassis);

// Ballistics Solver
aruwsrc::algorithms::OttoBallisticsSolver ballisticsSolver(
    &drivers()->visionCoprocessor,
    &odometryTracker,
    &turret,
    &frictionWheels,
    14.0, // defaultLaunchSpeed
    0);

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

// commands
// launcher commands
// base rotate/unjam commands
MoveIntegralCommand rotateAgitator(agitator, AGITATOR_ROTATE_CONFIG);

UnjamIntegralCommand unjamAgitator(agitator, AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
    *drivers(),
    agitator,
    rotateAgitator,
    unjamAgitator);

RefSystemProjectileLaunchedGovernor refSystemProjectileLaunchedGovernor(
    drivers()->refSerial,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

FrictionWheelsOnGovernor frictionWheelsOnGovernor(frictionWheels);

aruwsrc::control::agitator::ManualFireRateReselectionManager manualFireRateReselectionManager;
FireRateLimitGovernor fireRateLimitGovernor(manualFireRateReselectionManager);

GovernorLimitedCommand<3> rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched(
    {&agitator},
    rotateAndUnjamAgitator,
    {&refSystemProjectileLaunchedGovernor, &frictionWheelsOnGovernor, &fireRateLimitGovernor});

// rotates agitator with heat limiting applied
HeatLimitGovernor heatLimitGovernor(
    *drivers(),
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1,
    HEAT_LIMIT_BUFFER);
GovernorLimitedCommand<1> rotateAndUnjamAgitatorWithHeatLimiting(
    {&agitator},
    rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched,
    {&heatLimitGovernor});

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

cv::TurretCVCommand turretCVCommand(
    &drivers()->visionCoprocessor,
    &drivers()->controlOperatorInterface,
    &turret,
    &worldFrameYawTurretImuControllerCv,
    &worldFramePitchTurretImuControllerCv,
    &ballisticsSolver,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

aruwsrc::control::imu::ImuCalibrateCommand imuCalibrateCommand(
    drivers(),
    {{
        &getTurretMCBCanComm(),
        &turret,
        &chassisFrameYawTurretController,
        &chassisFramePitchTurretController,
        true,
    }},
    &chassis);

aruwsrc::chassis::BalancingChassisHomeCommand homeLegCommand(drivers(), &chassis);

user::TurretQuickTurnCommand turretUTurnCommand(&turret, M_PI);

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
    {&rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    true);
PressCommandMapping leftSwitchDown(
    drivers(),
    {&homeLegCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
PressCommandMapping leftSwitchUp(
    drivers(),
    {&imuCalibrateCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));
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
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    chassis.initialize();
    turret.initialize();
    frictionWheels.initialize();
    agitator.initialize();
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultBalstdCommands(Drivers *)
{
    chassis.setDefaultCommand(&autorotateDriveCommand);
    turret.setDefaultCommand(&turretUserWorldRelativeCommand);
    frictionWheels.setDefaultCommand(&spinFrictionWheels);
}

/* add any starting commands to the scheduler here --------------------------*/
void startBalstdCommands(Drivers *drivers) {}

/* register io mappings here ------------------------------------------------*/
void registerBalstdIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&bNotCtrlPressedRightSwitchDown);
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
