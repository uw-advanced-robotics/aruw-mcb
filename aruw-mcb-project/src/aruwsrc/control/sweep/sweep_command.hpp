/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef SWEEP_COMMAND_HPP_
#define SWEEP_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "sweep_subsystem.hpp"

namespace aruwsrc::control::sweep
{

class SweepCommand : public tap::control::Command
{
public:
    SweepCommand(SweepSubsystem* subsystem) : tap::control::Command(), subsystem(subsystem) {
        addSubsystemRequirement(subsystem);
    }

    void initialize() override;

    void execute() override;

    void end(bool) override { subsystem->motor->setDesiredOutput(0); }

    bool isFinished() const override { return currentFrequency > endFrequency; }

    const char* getName() const override { return "Sweeping"; }

private:
    SweepSubsystem* subsystem;

    float startTime = 0;

    float timeDifference = 0;

    float frequencySweepRate = 0.01;
    float startFrequency = 0, endFrequency = 250, currentFrequency;
    float setpoint = 0;

};  // class SweepCommand

}  // namespace aruwsrc::control::sweep
#endif  // SWEEP_COMMAND_HPP_
