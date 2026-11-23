/*
 * Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "damiao_rotate_slot_command.hpp"

#include <cmath>

#include "tap/architecture/clock.hpp"

namespace aruwsrc::motor_tester
{
DamiaoRotateSlotCommand::DamiaoRotateSlotCommand(
    control::motor::DamiaoSubsystem& subsystem,
    SlotReference& slotReference,
    int slotsToRotate)
    : subsystem(subsystem),
      slotReference(slotReference),
      slotsToRotate(slotsToRotate),
      slotAngle((2.0f * M_PI) / slotReference.slotCount)
{
    addSubsystemRequirement(&subsystem);
}

void DamiaoRotateSlotCommand::initialize()
{
    start = tap::arch::clock::getTimeMicroseconds();
    len = 0;
    DEBUG_launch_detected = false;
    const float position = subsystem.getEncoder().getPosition().getUnwrappedValue();
    if (!slotReference.initialized)
    {
        slotReference.origin = position;
        slotReference.initialized = true;
    }
    // Use the same single-slot grid for both commands, anchored to the initial encoder reading.
    // Treat an encoder reading just below a slot as already at that slot.
    const float slot =
        std::floor((position - slotReference.origin + POSITION_TOLERANCE) / slotAngle);
    // Keep the target unwrapped so advancing two slots never selects a shorter crossing.
    setpoint = slotReference.origin + (slot + slotsToRotate) * slotAngle;
    positionPid.reset();
    lastUpdateMillis = tap::arch::clock::getTimeMilliseconds();
    accelerating = true;
}

void DamiaoRotateSlotCommand::execute()
{
    const uint32_t now = tap::arch::clock::getTimeMilliseconds();
    const float dt = (now - lastUpdateMillis) * 0.001f;
    lastUpdateMillis = now;

    const auto& encoder = subsystem.getEncoder();
    if (encoder.isOnline())
    {
        const float error = setpoint - encoder.getPosition().getUnwrappedValue();
        if (error <= 0.0f)
        {
            accelerating = false;
        }
        subsystem.setDesiredOutput(
            accelerating ? control::motor::DM_J4340_LIMITS.maxOutputMilliNm
                         : static_cast<int32_t>(
                               positionPid.runController(error, encoder.getVelocity(), dt)));
    }
}

bool DamiaoRotateSlotCommand::pastSetpoint(){
  return !accelerating;
  
}

void DamiaoRotateSlotCommand::recordLaunchTime()
{
    if (DEBUG_launch_detected)
    {
        return;
    }
    DEBUG_launch_detected = true;
    len = tap::arch::clock::getTimeMicroseconds() - start;
}

bool DamiaoRotateSlotCommand::isReady() { return subsystem.getEncoder().isOnline(); }

void DamiaoRotateSlotCommand::end(bool) { subsystem.setDesiredOutput(0); }

}  // namespace aruwsrc::motor_tester
