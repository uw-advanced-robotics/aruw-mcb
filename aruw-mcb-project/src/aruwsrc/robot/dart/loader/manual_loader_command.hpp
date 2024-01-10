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

#ifndef MANUAL_LOADER_COMMAND_HPP_
#define MANUAL_LOADER_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "loader_subsystem.hpp"

typedef tap::communication::serial::Remote::Switch RemoteInputSwitch;
typedef tap::communication::serial::Remote::Channel RemoteInputChannel;

namespace aruwsrc::robot::dart
{
class ManualLoaderCommand : public tap::control::Command
{
    /**
     * A command whose purpose is to manually control the dart loading mechanism.
     * The control is meant to be done through a joystick.
     */
public:
    /**
     * @param drivers: Pointer to global drivers
     * @param loaderSubsystem: Subsystem for loading darts from the three magazines
     * @param remoteInputChannel: Analog input channel to control loader(s) from the remote
     */
    ManualLoaderCommand(
        tap::Drivers* drivers,
        LoaderSubsystem* loaderSubsystem,
        RemoteInputSwitch magazineSelectionSwitch,
        RemoteInputChannel remoteInputChannel);

    void initialize() override{};

    bool isReady() override { return true; };

    void execute() override;

    void end(bool) override { loaderSubsystem->stop(); }

    bool isFinished() const override { return false; }

    const char* getName() const override { return "Manual Loader"; }

    int32_t debugOutput;
    float outputGetterOutput = 0.0f;
    bool commandIsRunning = false, motorsOnline = false;
    uint16_t LOAD_SPEED = 3'000;

private:
    tap::Drivers* drivers;
    LoaderSubsystem* loaderSubsystem;
    RemoteInputSwitch magazineSelectionSwitch;
    RemoteInputChannel remoteInputChannel;
};

}  // namespace aruwsrc::robot::dart

#endif  // LAUNCHER_RELEASE_COMMAND_HPP_
