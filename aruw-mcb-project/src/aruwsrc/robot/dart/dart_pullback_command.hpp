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
#ifndef DART_PULLBACK_COMMAND_HPP_
#define DART_PULLBACK_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "dart_launcher_subsystem.hpp"

namespace aruwsrc::robot::dart
{
class DartPullbackCommand : public tap::control::Command
{
public:
    DartPullbackCommand(DartLauncherSubsystem &dartLauncher);

    void initialize() override;

    int32_t power = -5000;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char *getName() const override { return "DART PULLBACK"; }

private:
    DartLauncherSubsystem &dartLauncher;
};  // class DartPullbackCommand

}  // namespace aruwsrc::robot::dart
#endif  // DART_PULLBACK_COMMAND_HPP_
