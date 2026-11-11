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
DamiaoRotateSlotCommand::DamiaoRotateSlotCommand(control::motor::DamiaoSubsystem& subsystem)
    : subsystem(subsystem)
{
    addSubsystemRequirement(&subsystem);
}

void DamiaoRotateSlotCommand::initialize()
{
    const float angle = subsystem.getEncoder().getPosition().getWrappedValue();
    setpoint = (std::floor(angle / ROTATE_INCR) + 1.0f) * ROTATE_INCR;
    positionPid.reset();
    lastUpdateMillis = tap::arch::clock::getTimeMilliseconds();
}

void DamiaoRotateSlotCommand::execute()
{
    const auto& encoder = subsystem.getEncoder();
    if (!encoder.isOnline())
    {
        return;
    }

    const uint32_t now = tap::arch::clock::getTimeMilliseconds();
    const float dt = (now - lastUpdateMillis) * 0.001f;
    lastUpdateMillis = now;

    positionError = encoder.getPosition().minDifference(setpoint);
    subsystem.setDesiredOutput(
        static_cast<int32_t>(positionPid.runController(positionError, encoder.getVelocity(), dt)));
}

bool DamiaoRotateSlotCommand::isFinished() const
{
    // Stopping short would leave this same slot ahead on the next switch flick.
    return !subsystem.getEncoder().isOnline() ||
           (positionError <= 0.0f && positionError >= -POSITION_TOLERANCE);
}

void DamiaoRotateSlotCommand::end(bool) { subsystem.setDesiredOutput(0); }

}  // namespace aruwsrc::motor_tester
