/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "one_sided_motor_homing_command.hpp"

namespace aruwsrc::control
{
void OneSidedMotorHomingCommand::initialize() { subsystem.moveTowardLowerBound(); }

void OneSidedMotorHomingCommand::execute()
{
    if (trigger.isTriggered())
    {
        subsystem.setLowerBound();
        subsystem.setUpperBound();
    }
}

bool OneSidedMotorHomingCommand::isFinished() const { return subsystem.boundsSet(); }

void OneSidedMotorHomingCommand::end(bool) { subsystem.stop(); }

}  // namespace aruwsrc::control
