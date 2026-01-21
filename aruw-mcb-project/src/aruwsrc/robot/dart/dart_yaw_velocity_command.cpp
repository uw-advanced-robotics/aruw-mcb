/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "dart_yaw_velocity_command.hpp"

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/joint/homing/trigger_homed_joint_subsystem.hpp"
#include "aruwsrc/robot/dart/dart_constants.hpp"

namespace aruwsrc::robot::dart
{
DartYawVelocityCommand::DartYawVelocityCommand(
    tap::Drivers* drivers,
    TriggerHomedJointSubsystem* subsystem,
    Remote::Channel channel)
    : drivers(drivers),
      subsystem(subsystem),
      channel(channel)
{
    addSubsystemRequirement(subsystem);
}

void DartYawVelocityCommand::initialize() {}

void DartYawVelocityCommand::execute()
{
    subsystem->setSetpoint(
        subsystem->getPosition() +
        drivers->remote.getChannel(channel) * aruwsrc::dart::YAW_INPUT_SENSITIVITY);
}

bool DartYawVelocityCommand::isFinished() const
{
    return abs(drivers->remote.getChannel(channel)) < 0.05;
}

void DartYawVelocityCommand::end(bool) {}
}  // namespace aruwsrc::robot::dart