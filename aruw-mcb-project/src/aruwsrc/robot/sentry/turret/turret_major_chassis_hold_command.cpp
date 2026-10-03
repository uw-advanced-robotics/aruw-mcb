/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "turret_major_chassis_hold_command.hpp"

#include "tap/architecture/clock.hpp"

namespace aruwsrc::sentry::turret
{
TurretMajorChassisHoldCommand::TurretMajorChassisHoldCommand(
    aruwsrc::control::turret::YawTurretSubsystem& turretMajor,
    aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
        tap::algorithms::transforms::Axis::YAW>& chassisFrameYawController,
    float chassisYaw)
    : turretMajor(turretMajor),
      yawController(chassisFrameYawController),
      setpoint(chassisYaw)
{
    addSubsystemRequirement(&turretMajor);
}

void TurretMajorChassisHoldCommand::initialize()
{
    yawController.initialize();
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void TurretMajorChassisHoldCommand::execute()
{
    const uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    const float dt = (currTime - prevTime) / 1000.0f;
    prevTime = currTime;
    yawController.runController(dt, setpoint);
}

void TurretMajorChassisHoldCommand::end(bool) { turretMajor.getMutableMotor().setMotorOutput(0); }
}  // namespace aruwsrc::sentry::turret
