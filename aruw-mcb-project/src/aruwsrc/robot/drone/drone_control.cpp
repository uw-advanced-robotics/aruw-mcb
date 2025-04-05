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
#include "tap/control/command_mapper.hpp"
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"

#include "aruwsrc/algorithms/odometry/otto_kf_odometry_2d_subsystem.hpp"
#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"
#include "aruwsrc/control/agitator/constant_velocity_agitator_command.hpp"
#include "aruwsrc/control/agitator/constants/agitator_constants.hpp"
#include "aruwsrc/control/agitator/manual_fire_rate_reselection_manager.hpp"
#include "aruwsrc/control/agitator/unjam_spoke_agitator_command.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/governor/cv_on_target_governor.hpp"
#include "aruwsrc/control/governor/fire_rate_limit_governor.hpp"
#include "aruwsrc/control/governor/friction_wheels_on_governor.hpp"
#include "aruwsrc/control/governor/heat_limit_governor.hpp"
#include "aruwsrc/control/governor/ref_system_projectile_launched_governor.hpp"
#include "aruwsrc/control/launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/user/turret_user_control_command.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/drone/drone_drivers.hpp"
#include "aruwsrc/robot/drone/drone_turret_subsystem.hpp"

using namespace aruwsrc::drone;
using namespace aruwsrc::control;
using namespace aruwsrc::control::turret;
using namespace tap::control;
using namespace aruwsrc::control::turret::user;
using namespace aruwsrc::agitator;
using namespace aruwsrc::algorithms::odometry;
using namespace aruwsrc::control::agitator;
using namespace tap::control::setpoint;
using namespace tap::control::governor;

using namespace aruwsrc::control::governor;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace drone_control
{
inline aruwsrc::can::TurretMCBCanComm &getTurretMCBCanComm()
{
    return drivers()->turretMCBCanCommBus1;
}

/* define subsystems --------------------------------------------------------*/
tap::motor::DjiMotor pitchMotor(
    drivers(),
    PITCH_MOTOR_ID,
    CAN_BUS_PITCH_MOTOR,
    true,
    "Pitch Turret");

tap::motor::DjiMotor yawMotor(drivers(), YAW_MOTOR_ID, CAN_BUS_YAW_MOTOR, true, "Yaw Turret");

aruwsrc::control::turret::DroneTurretSubsystem turret(
    drivers(),
    &pitchMotor,
    &yawMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG,
    &getTurretMCBCanComm());

// transforms
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

/* define commands ----------------------------------------------------------*/
algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController(
    turret.pitchMotor,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
    turret.yawMotor,
    chassis_rel::YAW_PID_CONFIG);

TurretUserControlCommand turrettUserControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turret,
    &chassisFrameYawTurretController,
    &chassisFramePitchTurretController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR,
    0  // Assuming this is the desired turret ID
);

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

GovernorLimitedCommand<2> rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched(
    {&agitator},
    rotateAndUnjamAgitator,
    {&refSystemProjectileLaunchedGovernor, &frictionWheelsOnGovernor});

// rotates agitator with heat limiting applied
HeatLimitGovernor heatLimitGovernor(
    *drivers(),
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1,
    constants::HEAT_LIMIT_BUFFER);
GovernorLimitedCommand<1> rotateAndUnjamAgitatorWithHeatLimiting(
    {&agitator},
    rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched,
    {&heatLimitGovernor});

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

// Remote related mappings
HoldRepeatCommandMapping rightSwitchMiddle(
    drivers(),
    {&spinFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID),
    true);

HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&spinFrictionWheels, &rotateAndUnjamAgitatorWithHeatLimiting},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    true);

// Safe disconnect function
aruwsrc::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    turret.initialize();
    agitator.initialize();
    frictionWheels.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerDroneSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&frictionWheels);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultDroneCommands(Drivers *)
{
    turret.setDefaultCommand(&turrettUserControlCommand);
    frictionWheels.setDefaultCommand(&stopFrictionWheels);
}

/* add any starting commands to the scheduler here --------------------------*/
void startDroneCommands(Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&turrettUserControlCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerDroneIoMappings(Drivers *drivers)
{
    // Add IO mappings for control operator interface
    drivers->commandMapper.addMap(&rightSwitchMiddle);
    drivers->commandMapper.addMap(&rightSwitchUp);
}
}  // namespace drone_control

namespace aruwsrc::drone
{
void initSubsystemCommands(aruwsrc::drone::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &drone_control::remoteSafeDisconnectFunction);
    drone_control::initializeSubsystems();
    drone_control::registerDroneSubsystems(drivers);
    drone_control::setDefaultDroneCommands(drivers);
    drone_control::startDroneCommands(drivers);
    drone_control::registerDroneIoMappings(drivers);
}
}  // namespace aruwsrc::drone

#endif
