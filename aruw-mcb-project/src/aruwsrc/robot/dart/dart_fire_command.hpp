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

#ifndef DART_FIRE_COMMAND_HPP_
#define DART_FIRE_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "launcher/launcher_pull_subsystem.hpp"
#include "launcher/launcher_release_subsystem.hpp"

namespace aruwsrc::robot::dart
{
/**
 * A command whose purpose is to release the dart launcher and then relock it
 */
class DartFireCommand : public tap::control::Command
{
public:
    /**
     * @param drivers The drivers object
     * @param pivotSubsystem The pivot subsystem
     */
    DartFireCommand(
        tap::Drivers* drivers,
        LauncherPullSubsystem* launcherPullSubsystem,
        LauncherReleaseSubsystem* LauncherReleaseSubsystem)
        : drivers(drivers),
          launcherPullSubsystem(launcherPullSubsystem),
          launcherReleaseSubsystem(LauncherReleaseSubsystem)
    {
        addSubsystemRequirement(launcherPullSubsystem);
        addSubsystemRequirement(LauncherReleaseSubsystem);
    }

    void initialize() override {}

    bool isReady() override { return true; };

    void execute() override;

    void end(bool) override { launcherPullSubsystem->stop(); };

    bool isFinished() const override { return false; }

    const char* getName() const override { return "Dart loading command"; }

private:
    tap::Drivers* drivers;
    LauncherPullSubsystem* launcherPullSubsystem;
    LauncherReleaseSubsystem* launcherReleaseSubsystem;

    bool launched = false;

    tap::arch::PeriodicMilliTimer launchWaitTimer;
};

}  // namespace aruwsrc::robot::dart

#endif  // LAUNCHER_RELEASE_COMMAND_HPP_
