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

#include "output_sweep_command.hpp"

OutputSweepCommand::OutputSweepCommand(
    RawMotorSubsystem* subsystem,
    tap::gpio::Digital& digital,
    tap::gpio::Digital::OutputPin outputPin,
    int32_t minOutput,
    int32_t maxOutput,
    uint32_t levelLengthMillis,
    int32_t levelIncrement,
    int32_t dir)
    : motorSubsystem(subsystem),
      digital(digital),
      outputPin(outputPin),
      minOutput(minOutput),
      maxOutput(maxOutput),
      levelLengthMillis(levelLengthMillis),
      levelIncrement(levelIncrement),
      dir(dir)
{
    this->addSubsystemRequirement(subsystem);
}

void OutputSweepCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    if (!started)
    {
        started = true;
        startTime = currTime;
        endTime = (maxOutput - minOutput) / levelIncrement * levelLengthMillis + startTime;
    }

    uint8_t levelIndex = ((currTime - startTime) / levelLengthMillis);

    currentOutput = levelIndex * levelIncrement + minOutput;

    if (currentOutput > maxOutput) return;

    motorSubsystem->setDesiredOutput(currentOutput * dir);
    digital.set(outputPin, levelIndex % 2 == 0);
}

void OutputSweepCommand::end(bool) { motorSubsystem->stop(); }
