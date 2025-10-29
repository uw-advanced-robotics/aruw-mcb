/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef POSITION_SWEEP_COMMAND_HPP_
#define POSITION_SWEEP_COMMAND_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/gpio/digital.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "raw_motor_subsystem.hpp"

using namespace aruwsrc::characterizer;

class PositionSweepCommand : public tap::control::Command
{
public:
    explicit PositionSweepCommand(
        RawMotorSubsystem* subsystem,
        tap::gpio::Digital& digital,
        tap::gpio::Digital::OutputPin outputPin,
        int32_t minOutput,
        int32_t maxOutput,
        uint32_t levelLengthMillis,
        int32_t levelIncrement,
        int32_t dir = 1);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override { return currentOutput > maxOutput; }

    const char* getName() const override { return "output sweep"; }

private:
    RawMotorSubsystem* motorSubsystem;
    tap::gpio::Digital& digital;
    tap::gpio::Digital::OutputPin outputPin;
    int32_t minOutput, maxOutput;
    uint32_t levelLengthMillis;
    int32_t levelIncrement;
    int32_t dir{1};

    uint32_t startTime{0}, endTime;
    int32_t currentOutput{0};

};  // class PositionSweepCommand

#endif  // POSITION_SWEEP_COMMAND_HPP_
