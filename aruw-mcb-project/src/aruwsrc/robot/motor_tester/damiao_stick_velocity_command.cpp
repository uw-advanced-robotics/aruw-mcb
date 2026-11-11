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

#include "damiao_stick_velocity_command.hpp"

namespace aruwsrc::motor_tester
{
DamiaoStickVelocityCommand::DamiaoStickVelocityCommand(
    control::motor::DamiaoSubsystem& subsystem,
    tap::communication::serial::Remote& remote,
    tap::communication::serial::Remote::Channel channel,
    float maxVelocityRadPerSec)
    : subsystem(subsystem),
      remote(remote),
      channel(channel),
      maxVelocityRadPerSec(maxVelocityRadPerSec)
{
    addSubsystemRequirement(&subsystem);
}

void DamiaoStickVelocityCommand::execute()
{
    const auto position = subsystem.getEncoder().getPosition();
    DEBUG_wrappedPosition = position.getWrappedValue();
    DEBUG_unwrappedPosition = position.getUnwrappedValue();

    const float stick = remote.getChannel(channel);
    const float targetVelocity = (stick > -STICK_DEADZONE && stick < STICK_DEADZONE)
                                     ? 0.0f
                                     : stick * maxVelocityRadPerSec;
    subsystem.sendMitCommand(0.0f, targetVelocity, 0.0f, KD, 0.0f);
}

void DamiaoStickVelocityCommand::end(bool) { subsystem.stop(); }
}  // namespace aruwsrc::motor_tester
