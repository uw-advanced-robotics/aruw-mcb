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

#include "dart_pullback_command.hpp"

namespace aruwsrc::robot::dart
{
DartPullbackCommand::DartPullbackCommand(DartLauncherSubsystem &dartLauncher, int32_t desiredOutput)
    : dartLauncher(dartLauncher),
      desiredOutput(desiredOutput)
{
    addSubsystemRequirement(&dartLauncher);
}

void DartPullbackCommand::initialize() {}

void DartPullbackCommand::execute() { dartLauncher.moveMotor(desiredOutput); }

void DartPullbackCommand::end(bool) { dartLauncher.moveMotor(0); }

bool DartPullbackCommand::isFinished() const { return dartLauncher.isBeamBroken(); }

}  // namespace aruwsrc::robot::dart