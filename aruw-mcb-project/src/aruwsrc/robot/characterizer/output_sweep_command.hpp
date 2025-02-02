/*
 * Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef OUTPUT_SWEEP_COMMAND_HPP_
#define OUTPUT_SWEEP_COMMAND_HPP_

#include "tap/communication/gpio/digital.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "raw_motor_subsystem.hpp"

using namespace aruwsrc::characterizer;

class OutputSweepCommand : public tap::control::Command
{
public:
    explicit OutputSweepCommand(
        RawMotorSubsystem* subsystem,
        tap::gpio::Digital& digital,
        tap::gpio::Digital::OutputPin outputPin,
        int32_t minOutput,
        int32_t maxOutput,
        uint32_t levelLengthMillis,
        int32_t levelIncrement,
        int32_t dir = 1);

    void initialize() override { started = false; }

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

    bool started{false};
    uint32_t startTime{0}, endTime;
    int32_t currentOutput{0};

};  // class OutputSweepCommand

#endif  // OUTPUT_SWEEP_COMMAND_HPP_
