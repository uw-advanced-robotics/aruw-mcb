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

#include "turret_pid_control_algorithms.hpp"

namespace aruwsrc::control::turret
{
void runSinglePidPitchChassisFrameController(
    const float dt,
    const float userInput,
    tap::algorithms::SmoothPid &pid,
    TurretSubsystem *turretSubsystem)
{
    // limit the yaw min and max angles
    turretSubsystem->setPitchSetpoint(turretSubsystem->getPitchSetpoint() + userInput);

    // position controller based on turret pitch gimbal
    float positionControllerError =
        turretSubsystem->getCurrentPitchValue().difference(turretSubsystem->getPitchSetpoint());

    float pidOutput =
        pid.runController(positionControllerError, turretSubsystem->getPitchVelocity(), dt);

    turretSubsystem->setPitchMotorOutput(pidOutput);
}
}  // namespace aruwsrc::control::turret
