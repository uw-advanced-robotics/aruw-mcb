/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "rotate_magazine_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/command.hpp"

#include "dart_constants.hpp"
namespace aruwsrc::dart
{
RotateMagazineCommand::RotateMagazineCommand(DartReloaderSubsystem& subsystem)
    : subsystem(subsystem)
{
    addSubsystemRequirement(&subsystem);
}

void RotateMagazineCommand::initialize()
{
    float index = floor(
        (subsystem.getPosition() - DART_MAGAZINE_RELOAD_POSITION) / DART_MAGAZINE_ROTATE_INCREMENT);
    // if index is not near its int version, then we must be in some unknown position
    // in this case, we go to the nearest "free" position
    if (!tap::algorithms::compareFloatClose(
            ((subsystem.getPosition() - DART_MAGAZINE_RELOAD_POSITION) /
             DART_MAGAZINE_ROTATE_INCREMENT),
            index,
            0.01f))
    {
        // TODO: tune how close we need to be to the nearest position
        // TODO: chekc if we actually need to go to the nearest free or if we can just go to
        // the initial free
        subsystem.setSetpoint(DART_MAGAZINE_FREE_POSITION);
    }
    else
    {  // otherwise, we just increment one index
        subsystem.setSetpoint(subsystem.getSetpoint() + DART_MAGAZINE_ROTATE_INCREMENT);
    }
}
void RotateMagazineCommand::execute() {}
void RotateMagazineCommand::end(bool) {}
bool RotateMagazineCommand::isFinished() const { return subsystem.atSetpoint(); }
}  // namespace aruwsrc::dart
