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

#ifndef MANUAL_LAUNCHER_RELEASE_COMMAND_HPP_
#define MANUAL_LAUNCHER_RELEASE_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "launcher_release_subsystem.hpp"

using namespace tap::communication::serial;

typedef Remote::Switch RemoteInputSwitch;

namespace aruwsrc::robot::dart
{
/**
 * A command whose primary purpose is to control the release mechanism of the dart launcher.
 * The command is controlled by a switch on the remote, where up is release, middle is stop, and
 * down is to relock with the pulling mechanism.
 */
class ManualLauncherReleaseCommand : public tap::control::Command
{
public:
    /**
     * @param drivers: Pointer to global drivers
     * @param launcherReleaseSubsystem: Subsystem to engage and disengage the winch motor
     * @param remoteInputSwitch: Remote input switch to control the launcher release subsystem from the remote
     */
    ManualLauncherReleaseCommand(
        tap::Drivers* drivers,
        LauncherReleaseSubsystem* launcherReleaseSubsystem,
        RemoteInputSwitch remoteInputSwitch);

    void initialize() override{};

    bool isReady() override { return true; };

    void execute() override;

    void end(bool) override { launcherReleaseSubsystem->stop(); }

    bool isFinished() const override { return false; }

    const char* getName() const override { return "Launcher Release"; }

private:
    tap::Drivers* drivers;
    LauncherReleaseSubsystem* launcherReleaseSubsystem;
    RemoteInputSwitch remoteInputSwitch;
};

}  // namespace aruwsrc::robot::dart

#endif  // LAUNCHER_RELEASE_COMMAND_HPP_
