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

#if defined(TARGET_DRONE)

#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/setpoint/commands/unjam_integral_command.hpp"

#include "agitator/constants/agitator_constants.hpp"
#include "agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/otto_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/cv/turret_cv_command.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "client-display/client_display_command.hpp"
#include "client-display/client_display_subsystem.hpp"
#include "drone/drone_subsystem.hpp"
#include "governor/cv_on_target_governor.hpp"
#include "governor/fire_rate_limit_governor.hpp"
#include "governor/friction_wheels_on_governor.hpp"
#include "governor/heat_limit_governor.hpp"
#include "governor/ref_system_projectile_launched_governor.hpp"
#include "imu/imu_calibrate_command.hpp"
#include "launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "turret/algorithms/chassis_frame_turret_controller.hpp"
#include "turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "turret/constants/turret_constants.hpp"
#include "turret/drone_turret_subsystem.hpp"
#include "turret/user/turret_quick_turn_command.hpp"
#include "turret/user/turret_user_world_relative_command.hpp"

using namespace aruwsrc::agitator;
using namespace aruwsrc::control::agitator;
using namespace aruwsrc::control::client_display;
using namespace aruwsrc::control::turret;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
aruwsrc::driversFunc drivers = aruwsrc::DoNotUse_getDrivers;

namespace drone_control
{
inline aruwsrc::can::TurretMCBCanComm &getTurretMCBCanComm()
{
    return drivers()->turretMCBCanCommBus1;
}

// most of this is being copied over from standard, apologies in advance if it's wrong
/* define subsystems--------------------------------------------------------*/

DroneSubsystem droneTelemChassis(drivers());

tap::motor::DjiMotor pitchMotor(drivers(), PITCH_MOTOR_ID, CAN_BUS_MOTORS, false, "Pitch Turret");

tap::motor::DjiMotor yawMotor(drivers(), YAW_MOTOR_ID, CAN_BUS_MOTORS, true, "Yaw Turret");

DroneTurretSubsystem turret(
    drivers(),
    &pitchMotor,
    &yawMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG,
    &getTurretMCBCanComm());

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

OttoKFOdometry2DSubsystem odometrySubsystem(*drivers(), turret, droneChassissTelem);

OttoBallisticsSolver ballisticsSolver(
    *drivers(),
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
    *drivers(),
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

/* define commands ----------------------------------------------------------*/

// Safe disconnect function
aruwsrc::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems() {}

/* register subsystems here -------------------------------------------------*/
void registerDroneSubsystems(aruwsrc::Drivers *) {}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultDroneCommands(aruwsrc::Drivers *) {}

/* add any starting commands to the scheduler here --------------------------*/
void startDroneCommands(aruwsrc::Drivers *) {}

/* register io mappings here ------------------------------------------------*/
void registerDroneIoMappings(aruwsrc::Drivers *) {}
}  // namespace drone_control

namespace aruwsrc::control
{
void initSubsystemCommands(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &drone_control::remoteSafeDisconnectFunction);
    drone_control::initializeSubsystems();
    drone_control::registerDroneSubsystems(drivers);
    drone_control::setDefaultDroneCommands(drivers);
    drone_control::startDroneCommands(drivers);
    drone_control::registerDroneIoMappings(drivers);
}
}  // namespace aruwsrc::control

#endif
