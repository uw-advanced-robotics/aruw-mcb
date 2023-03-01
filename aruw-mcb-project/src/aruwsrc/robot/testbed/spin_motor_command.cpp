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

#ifdef TARGET_TESTBED
#include "spin_motor_command.hpp"

namespace aruwsrc::testbed
{
SpinMotorCommand::SpinMotorCommand(
    tap::Drivers* drivers,
    TMotorSubsystem* tmotorSubsystem,
    int16_t speed)
    : drivers(drivers),
      tmotorSubsystem(tmotorSubsystem),
      speed(speed)
{
    addSubsystemRequirement(tmotorSubsystem);
}

void SpinMotorCommand::initialize() { tmotorSubsystem->setDesiredOutput(speed); }

void SpinMotorCommand::execute()
{
    tmotorSubsystem->setDesiredOutput(speed);
    return;
}

void SpinMotorCommand::end(bool) { tmotorSubsystem->setDesiredOutput(0); }

bool SpinMotorCommand::isFinished() const { return false; }
}  // namespace aruwsrc::testbed
#endif  // TARGET_TESTBED
