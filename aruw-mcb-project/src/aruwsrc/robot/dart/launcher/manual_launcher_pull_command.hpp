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

#ifndef MANUAL_LAUNCHER_PULL_COMMAND_HPP_
#define MANUAL_LAUNCHER_PULL_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "launcher_pull_subsystem.hpp"

using namespace tap::communication::serial;

typedef tap::communication::serial::Remote::Channel RemoteInputChannel;  // Pointer to a switch state

namespace aruwsrc::robot::dart
{
/**
 * A command whose primary job it is to control the dart launcher pulling mechanism.
 * The control of this command is meant to be used through one of the switches on the remote, with
 * down being pull, middle being stop, and up being push up.
 */
class ManualLauncherPullCommand : public tap::control::Command
{
public:
    /**
     * @param drivers Pointer to robot driver
     * @param launcherPullSubsystem Dart launcher subsystem
     * @param remoteInputChannel Analog input channel to control the pull mechanism
     */
    ManualLauncherPullCommand(
        tap::Drivers* drivers,
        LauncherPullSubsystem* launcherPullSubsystem,
        RemoteInputChannel remoteInputChannel);

    void initialize() override{};

    bool isReady() override { return true; };

    void execute() override;

    void end(bool) override { launcherPullSubsystem->stop(); }

    bool isFinished() const override { return false; }

    const char* getName() const override { return "Launcher Pull Command"; }

    const uint16_t MAX_PULL_SPEED = 8'000;

private:
    tap::Drivers* drivers;
    LauncherPullSubsystem* launcherPullSubsystem;
    RemoteInputChannel remoteInputChannel;
};

}  // namespace aruwsrc::robot::dart

#endif  // MANUAL_LAUNCHER_PULL_COMMAND_HPP_
