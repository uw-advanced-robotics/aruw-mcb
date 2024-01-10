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

#ifndef GO_TO_PULL_SETPOINT_COMMAND_HPP_
#define GO_TO_PULL_SETPOINT_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "launcher_pull_subsystem.hpp"

namespace aruwsrc::robot::dart
{
/**
 * A command whose purpose is to make the pull subsystem go to a point. Meant ot go to launch
 * position
 */
class GoToPullSetpointCommand : public tap::control::Command
{
public:
    /**
     * @param drivers The drivers object
     * @param launcherPullSubsystem The launcher subsystem
     */
    GoToPullSetpointCommand(
        tap::Drivers* drivers,
        LauncherPullSubsystem* launcherPullSubsystem,
        float targetPosition)
        : drivers(drivers),
          launcherPullSubsystem(launcherPullSubsystem),
          targetPosition(targetPosition)
    {
        addSubsystemRequirement(launcherPullSubsystem);
    }

    void initialize() override{};

    bool isReady() override { return true; };

    void execute() override { launcherPullSubsystem->setSetpoint(targetPosition); };

    void end(bool) override { launcherPullSubsystem->stop(); }

    bool isFinished() const override { return launcherPullSubsystem->atSetpoint(); }

    const char* getName() const override { return "Go to pull setpoint Command"; }

private:
    tap::Drivers* drivers;
    LauncherPullSubsystem* launcherPullSubsystem;
    float targetPosition;
};

}  // namespace aruwsrc::robot::dart

#endif  // LAUNCHER_RELEASE_COMMAND_HPP_