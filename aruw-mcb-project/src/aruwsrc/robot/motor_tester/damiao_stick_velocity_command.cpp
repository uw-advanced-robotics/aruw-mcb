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
    tap::communication::serial::Remote& remote)
    : subsystem(subsystem),
      remote(remote)
{
    addSubsystemRequirement(&subsystem);
}

void DamiaoStickVelocityCommand::execute()
{
    const float stick = remote.getChannel(
        tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
    const float targetVelocity = (stick > -STICK_DEADZONE && stick < STICK_DEADZONE)
                                     ? 0.0f
                                     : stick * MAX_VELOCITY_RAD_PER_SEC;
    subsystem.sendMitCommand(0.0f, targetVelocity, 0.0f, KD, 0.0f);
}

void DamiaoStickVelocityCommand::end(bool) { subsystem.stop(); }
}  // namespace aruwsrc::motor_tester
