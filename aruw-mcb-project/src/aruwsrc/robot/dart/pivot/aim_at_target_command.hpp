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

#ifndef PIVOT_AIM_AT_TARGET_COMMAND_HPP_
#define PIVOT_AIM_AT_TARGET_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "pivot_subsystem.hpp"

namespace aruwsrc::robot::dart
{
/**
 * A command whose purpose is to make the pivot subsystem aim at something
 */
class PivotAimAtTargetCommand : public tap::control::Command
{
public:
    /**
     * @param drivers The drivers object
     * @param pivotSubsystem The pivot subsystem
     */
    PivotAimAtTargetCommand(
        tap::Drivers* drivers,
        PivotSubsystem* pivotSubsystem,
        float targetPosition)
        : drivers(drivers),
          pivotSubsystem(pivotSubsystem),
          targetPosition(targetPosition)
    {
        addSubsystemRequirement(pivotSubsystem);
    }

    void initialize() override{};

    bool isReady() override { return true; };

    void execute() override { pivotSubsystem->setSetpoint(targetPosition); };

    void end(bool) override { pivotSubsystem->stop(); }

    bool isFinished() const override { return pivotSubsystem->atSetpoint(); }

    const char* getName() const override { return "Pivot Aim aT Target Command"; }

private:
    tap::Drivers* drivers;
    PivotSubsystem* pivotSubsystem;
    float targetPosition;
};

}  // namespace aruwsrc::robot::dart

#endif  // LAUNCHER_RELEASE_COMMAND_HPP_
