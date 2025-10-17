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

#include "turret_user_control_command.hpp"

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/drivers.hpp"

#include "../turret_subsystem.hpp"
#include "aruwsrc/robot/control_operator_interface.hpp"

using tap::algorithms::WrappedFloat;

namespace aruwsrc::control::turret::user
{
// STEP 1 (Turret User Control): Complete constructor
TurretUserControlCommand::TurretUserControlCommand(
    tap::Drivers *drivers,
    ControlOperatorInterface &controlOperatorInterface,
    TurretSubsystem *turretSubsystem,
    algorithms::TurretYawControllerInterface *yawController,
    algorithms::TurretPitchControllerInterface *pitchController,
    float userYawInputScalar,
    float userPitchInputScalar,
    uint8_t turretID)
    : drivers(drivers),
      controlOperatorInterface(controlOperatorInterface),
      turretSubsystem(turretSubsystem),
      yawController(yawController),
      pitchController(pitchController),
      userYawInputScalar(userYawInputScalar),
      userPitchInputScalar(userPitchInputScalar),
      turretID(turretID)
{
    // TODO: Register subsystem requirement
    // CALL the inherited method that registers a subsystem requirement
    // PASS the turret subsystem pointer as the argument
    // This prevents other commands from using the turret while this command runs
}

// STEP 4A (Turret User Control): Implement isReady
bool TurretUserControlCommand::isReady() {
    // TODO: Check if command can safely execute
    // IF command is not finished AND controllers are online:
    //     RETURN true (ready to run)
    // ELSE:
    //     RETURN false (not ready)
    // Controllers have an isOnline() method to check hardware status
}

// STEP 2 (Turret User Control): Implement initialize method
void TurretUserControlCommand::initialize()
{
    // TODO: Initialize both controllers
    // FOR each controller (yaw and pitch):
    // CALL the controller's initialize method
    // This resets internal state like accumulated error and previous setpoints
    
    // SET prevTime to current system time in milliseconds
    // CALL the clock utility function that returns current time
    // This gives execute() a starting point for calculating time deltas
}

// STEP 3 (Turret User Control): Implement execute method  
void TurretUserControlCommand::execute()
{
    // TODO: Calculate time delta
    // DECLARE current_time = get current system time in milliseconds
    // CALCULATE dt = current_time - prevTime 
    // UPDATE prevTime = current_time for next iteration
    
    // TODO: Get user input from operator interface
    // DECLARE yaw_input = get turret yaw input for this turret ID
    // DECLARE pitch_input = get turret pitch input for this turret ID
    // The operator interface returns processed, scaled input values
    
    // TODO: Calculate new setpoints based on current position + user input
    // DECLARE yaw_setpoint = current yaw setpoint + (yaw_sensitivity * yaw_input)
    // DECLARE pitch_setpoint = current pitch setpoint + (pitch_sensitivity * pitch_input)
    // Use WrappedFloat for angle arithmetic to handle wrapping
    
    // TODO: Command controllers to track new setpoints
    // FOR each controller (yaw and pitch):
    //     CALL runController with time delta and new setpoint
    //     This updates motor outputs to track the desired position
}

// STEP 4B (Turret User Control): Implement isFinished  
bool TurretUserControlCommand::isFinished() const
{
    // TODO: Determine if command should stop
    // This is a manual control command that runs indefinitely
    // IF any controller is offline (hardware failure):
    //     RETURN true (command should stop for safety)
    // ELSE:
    //     RETURN false (continue running)
    // Check both yaw and pitch controllers for hardware status
}

// STEP 4C (Turret User Control): Implement end
void TurretUserControlCommand::end(bool)
{
    // TODO: Clean up when command ends
    // This method ensures the robot is left in a safe state
    // You might want to:
    // - Stop motor outputs by setting them to zero
    // - Reset controller states
    // - Log why the command ended (normal vs interrupted)
    
    // The 'interrupted' parameter tells you if the command was forcibly stopped
    // interrupted == true: Another command took control
    // interrupted == false: Command finished naturally (unlikely for user control)
}

}  // namespace aruwsrc::control::turret::user
