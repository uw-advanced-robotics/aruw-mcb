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

#ifndef DART_COMMAND_HPP_
#define DART_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/robot/dart/dart_subsystem.hpp"

namespace aruwsrc::dart
{
class DartCommand : public tap::control::Command
{
public:
    DartCommand(aruwsrc::dart::DartSubsystem& dartSubsystem, tap::Drivers* drivers);

    void initialize() override {}

    void execute() override;

    bool isFinished() const override { return true; }

    void end(bool interrupt) override;

    const char* getName() const override { return "Dart command"; }

private:
    aruwsrc::dart::DartSubsystem& dartSubsystem;
    tap::Drivers* drivers;
};

}  // namespace aruwsrc::dart

#endif
