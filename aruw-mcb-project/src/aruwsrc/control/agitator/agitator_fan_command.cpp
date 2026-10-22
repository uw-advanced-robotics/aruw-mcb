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

#include "agitator_fan_command.hpp"

namespace aruwsrc::control::agitator
{
AgitatorFanCommand::AgitatorFanCommand(
    tap::Drivers* drivers,
    AgitatorFanSubsystem& fan,
    float onDuty,
    float offDuty,
    const tap::control::Command* disableWhenScheduled)
    : drivers(drivers),
      fan(fan),
      onDuty(onDuty),
      offDuty(offDuty),
      disableWhenScheduled(disableWhenScheduled)
{
    addSubsystemRequirement(&fan);
}

void AgitatorFanCommand::initialize() { updateFanDuty(); }

void AgitatorFanCommand::execute() { updateFanDuty(); }

void AgitatorFanCommand::end(bool) { setFanDuty(offDuty); }

void AgitatorFanCommand::updateFanDuty() { setFanDuty(shouldRunFan() ? onDuty : offDuty); }

bool AgitatorFanCommand::shouldRunFan() const
{
    return disableWhenScheduled == nullptr ||
           !drivers->commandScheduler.isCommandScheduled(disableWhenScheduled);
}

void AgitatorFanCommand::setFanDuty(float duty) { fan.setFanDuty(duty); }

}  // namespace aruwsrc::control::agitator
