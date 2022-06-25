/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SCOPED_SMOOTH_PID_CONFIGURATION_COMMAND
#define SCOPED_SMOOTH_PID_CONFIGURATION_COMMAND

#include "tap/control/command.hpp"
#include "tap/algorithms/smooth_pid.hpp"

#include <optional>

using namespace tap::algorithms;

namespace aruwsrc
{
namespace control
{
class ScopedSmoothPidConfigurationCommand : public tap::control::Command
{
public:
    explicit ScopedSmoothPidConfigurationCommand(Command *innerCommand, SmoothPid* pid, SmoothPidConfig *pidConfigWhileRunning);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "scoped smooth pid configuration"; }

private:
    Command *innerCommand;
    SmoothPid* subsystepid;
    SmoothPidConfig *pidConfigWhileRunning;

    std::optional<SmoothPidConfig> oldConfig = std::nullopt;
};  // class ScopedSmoothPidConfigurationCommand

}  // namespace control

}  // namespace aruwsrc

#endif  // SCOPED_SMOOTH_PID_CONFIGURATION_COMMAND
