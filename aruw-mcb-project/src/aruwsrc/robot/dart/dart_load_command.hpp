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

#ifndef DART_LOAD_COMMAND_HPP_
#define DART_LOAD_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "launcher/launcher_pull_subsystem.hpp"
#include "loader/loader_subsystem.hpp"

namespace aruwsrc::robot::dart
{
struct DartLoadingPositionAndPullback
{
    LoadingMotor loadingPosition;
    float pullingPositionOfLauncher;
    float pullingPositionOfHook;
    float loaderPositionOfHook;
};

typedef float (*PullBackLocation)();

enum DartLoadingState
{
    UNSET,
    PULLED_TO_LOADER_BACK,
    LOADER_DOWN,
    HOOKED,
    LOADER_UP,
    PULLED_BACK_TO_FIRE
};

/**
 * A command whose purpose is to make the dart load from a specific loader
 */
class DartLoadCommand : public tap::control::Command
{
public:
    /**
     * @param drivers The drivers object
     * @param pivotSubsystem The pivot subsystem
     */
    DartLoadCommand(
        tap::Drivers* drivers,
        LauncherPullSubsystem* launcherPullSubsystem,
        LoaderSubsystem* loaderSubsystem,
        DartLoadingPositionAndPullback loadingPositionAndPullback,
        PullBackLocation pullBackLocation)
        : drivers(drivers),
          launcherPullSubsystem(launcherPullSubsystem),
          loaderSubsystem(loaderSubsystem),
          loadingPositionAndPullback(loadingPositionAndPullback),
          pullBackLocation(pullBackLocation)
    {
        addSubsystemRequirement(launcherPullSubsystem);
        addSubsystemRequirement(loaderSubsystem);
    }

    void initialize() override { currentState = UNSET; }

    bool isReady() override { return true; };

    void execute() override;

    void end(bool) override
    {
        launcherPullSubsystem->stop();
        loaderSubsystem->stop();
    };

    bool isFinished() const override { return false; }

    const char* getName() const override { return "Dart loading command"; }

    inline void setLoaderMotor(LoadingMotor motor, float setpoint)
    {
        switch (motor)
        {
            case TOP:
                loaderSubsystem->setSetpoint(setpoint, 0, 0);
                break;
            case MIDDLE:
                loaderSubsystem->setSetpoint(0, setpoint, 0);
                break;
            case BOTTOM:
                loaderSubsystem->setSetpoint(0, 0, setpoint);
                break;
            default:
                break;
        }
    }

private:
    tap::Drivers* drivers;
    LauncherPullSubsystem* launcherPullSubsystem;
    LoaderSubsystem* loaderSubsystem;

    DartLoadingPositionAndPullback loadingPositionAndPullback;
    PullBackLocation pullBackLocation;

    DartLoadingState currentState;
};

}  // namespace aruwsrc::robot::dart

#endif  // LAUNCHER_RELEASE_COMMAND_HPP_
