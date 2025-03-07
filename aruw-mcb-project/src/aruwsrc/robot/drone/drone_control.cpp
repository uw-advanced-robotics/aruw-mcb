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
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/robot/drone/drone_drivers.hpp"
#include "aruwsrc/robot/drone/drone_turret_subsystem.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "tap/control/command_mapper.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/user/turret_user_control_command.hpp"
using namespace aruwsrc::drone;
using namespace aruwsrc::control;
using namespace aruwsrc::control::turret;
using namespace tap::control;
using namespace aruwsrc::control::turret::user;


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


// Safe disconnect function
aruwsrc::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());


/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems() {
    turret.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerDroneSubsystems(Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&turret);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultDroneCommands(Drivers *drivers) {
    turret.setDefaultCommand(&turrettUserControlCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startDroneCommands(Drivers *drivers) {
    drivers->commandScheduler.addCommand(&turrettUserControlCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerDroneIoMappings(Drivers *drivers) {
    // Add IO mappings for control operator interface
    
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
