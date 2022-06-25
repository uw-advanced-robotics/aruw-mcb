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

#include "scoped_smooth_pid_configuration_command.hpp"

#include "tap/control/subsystem.hpp"

namespace aruwsrc
{
namespace control
{
ScopedSmoothPidConfigurationCommand::ScopedSmoothPidConfigurationCommand(Command *innerCommand, SmoothPid* pid, SmoothPidConfig *pidConfigWhileRunning)
    : Command(),
      innerCommand(innerCommand),
      pid(pid),
      pidConfigWhileRunning(pidConfigWhileRunning)
{
}

void ScopedSmoothPidConfigurationCommand::initialize() {
    oldConfig = pid->getConfig();
    pid->setConfig(pidConfigWhileRunning);
    pid->reset();
    innerCommand->initialize();
}

void ScopedSmoothPidConfigurationCommand::execute() {
    innerCommand->execute();
}

void ScopedSmoothPidConfigurationCommand::end(bool interrupted) {
    if (oldConfig.has_value()) {
        pid->setConfig(oldConfig);
        pid.reset();
    }

    innerCommand->end(interrupted);
}

bool ScopedSmoothPidConfigurationCommand::isFinished() const { return innerCommand->isFinished(); }
}  // namespace control

}  // namespace aruwsrc
