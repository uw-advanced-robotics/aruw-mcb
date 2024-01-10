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

#ifndef MANUAL_PIVOT_COMMAND_HPP_
#define MANUAL_PIVOT_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "pivot_subsystem.hpp"

typedef tap::communication::serial::Remote::Channel RemoteInputChannel;

namespace aruwsrc::robot::dart
{
/**
 * A command whose purpose is to manually control the pivot subsystem.
 * The control is meant to be done through a joystick.
 */
class ManualPivotCommand : public tap::control::Command
{
public:
    /**
     * @param drivers The drivers object
     * @param pivotSubsystem The pivot subsystem
     * @param remoteInputChannel Analog input channel from the remote to control the pivot subsystem
     */
    ManualPivotCommand(
        tap::Drivers* drivers,
        PivotSubsystem* pivotSubsystem,
        RemoteInputChannel remoteInputChannel);

    void initialize() override {}

    void execute() override;

    void end(bool) override { pivotSubsystem->stop(); }

    bool isFinished() const override { return false; }

    const char* getName() const override { return "Manual Pivot Command"; }

    uint16_t MANUAL_DRIVE_SPEED_MAG = 1000.0f;

private:
    tap::Drivers* drivers;
    PivotSubsystem* pivotSubsystem;
    RemoteInputChannel remoteInputChannel;
};

}  // namespace aruwsrc::robot::dart

#endif  // LAUNCHER_RELEASE_COMMAND_HPP_
