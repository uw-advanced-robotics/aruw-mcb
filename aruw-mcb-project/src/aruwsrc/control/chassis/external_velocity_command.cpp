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

#include "external_velocity_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"

using tap::algorithms::limitVal;

namespace aruwsrc::control::chassis
{
ExternalVelocityCommand::ExternalVelocityCommand(
    HolonomicChassisSubsystem& chassis,
    const aruwsrc::communication::serial::VisionCoprocessor& visionCoprocessor,
    float wheelRadius,
    float wheelbaseRadius)
    : chassis(chassis),
      visionCoprocessor(visionCoprocessor),
      // same conversion ChassisAutoNavController uses for translation
      metersPerSecondToWheelRpm(60.0f / (M_TWOPI * wheelRadius)),
      // r is in wheel rpm; rim speed of a wheel is omega * wheelbaseRadius
      radiansPerSecondToWheelRpm(60.0f * wheelbaseRadius / (M_TWOPI * wheelRadius))
{
    addSubsystemRequirement(&chassis);
}

void ExternalVelocityCommand::initialize() { chassis.setZeroRPM(); }

void ExternalVelocityCommand::execute()
{
    const uint32_t lastReceived = visionCoprocessor.getLastChassisVelocityCommandTimeMs();
    const uint32_t now = tap::arch::clock::getTimeMilliseconds();

    if (lastReceived == 0 || now - lastReceived > COMMAND_TIMEOUT_MS ||
        !visionCoprocessor.isCvOnline() || !chassis.allMotorsOnline())
    {
        chassis.setZeroRPM();
        return;
    }

    const auto& command = visionCoprocessor.getLastChassisVelocityCommand();
    const float vx = limitVal(command.vx, -MAX_TRANSLATION_SPEED, MAX_TRANSLATION_SPEED);
    const float vy = limitVal(command.vy, -MAX_TRANSLATION_SPEED, MAX_TRANSLATION_SPEED);
    const float wz = limitVal(command.wz, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

    chassis.setDesiredOutput(
        vx * metersPerSecondToWheelRpm,
        vy * metersPerSecondToWheelRpm,
        wz * radiansPerSecondToWheelRpm);
}

void ExternalVelocityCommand::end(bool) { chassis.setZeroRPM(); }
}  // namespace aruwsrc::control::chassis
