/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef KILL_ALL_COMMAND_HPP_
#define KILL_ALL_COMMAND_HPP_

#include <aruwlib/control/command.hpp>
#include <modm/processing/timer/timeout.hpp>

#include "KillAllSubsystem.hpp"

using namespace aruwlib::control;

namespace aruwlib
{
namespace control
{
class KillAllCommand : public aruwlib::control::Command
{
public:
    KillAllCommand(KillAllSubsystem *killAllSubsystem, aruwlib::Drivers *drivers);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char *getName() const { return "kill all command"; }

private:
    KillAllSubsystem *killAllSubsystem;

    aruwlib::Drivers *drivers;
};  // class KillAllCommand
}  // namespace control
}  // namespace aruwlib

#endif  // KILL_ALL_COMMAND_HPP_
