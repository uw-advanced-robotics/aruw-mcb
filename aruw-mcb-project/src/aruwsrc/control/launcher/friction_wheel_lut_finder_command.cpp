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

#include "friction_wheel_lut_finder_command.hpp"

#include "friction_wheel_interface.hpp"

namespace aruwsrc::control::launcher
{
FrictionWheelLUTFinderCommand::FrictionWheelLUTFinderCommand(FrictionWheelInterface *subsystem)
    : subsystem(subsystem)
{
    this->addSubsystemRequirement(subsystem);
}

void FrictionWheelLUTFinderCommand::initialize()
{
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void FrictionWheelLUTFinderCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    if (currTime - prevTime > TIME_INC_MILLI)
    {
        curRPM += RPM_INCREMENT;
        this->subsystem->setDesiredLaunchSpeed(curRPM, true);
        prevTime = currTime;
    }
}

void FrictionWheelLUTFinderCommand::end(bool) { this->subsystem->setDesiredLaunchSpeed(0.0, true); }

bool FrictionWheelLUTFinderCommand::isFinished() const { return curRPM >= RPM_MAX; }

}  // namespace aruwsrc::control::launcher