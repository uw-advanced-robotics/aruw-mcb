/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#if defined(TARGET_SENTINEL)

#ifndef SENTINEL_RANDOM_DRIVE_COMMAND_HPP_
#define SENTINEL_RANDOM_DRIVE_COMMAND_HPP_

#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"

#include "sentinel_drive_subsystem.hpp"

namespace aruwsrc::control::sentinel::drive
{
class SentinelRandomDriveCommand : public tap::control::Command
{
public:
    explicit SentinelRandomDriveCommand(SentinelDriveSubsystem* subsystem);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "sentinel random drive"; }

private:
    static const int16_t MIN_RPM = 5000;
    static const int16_t MAX_RPM = 7000;
    static const int16_t CHANGE_TIME_INTERVAL = 750;
    static constexpr float TURNAROUND_BUFFER = 0.25f * SentinelDriveSubsystem::RAIL_LENGTH;

    float currentRPM = 0;
    bool chosenNewRPM = false;
    float newPos = 0;
    float oldPos = 0;
    bool prevEvade = true;

    SentinelDriveSubsystem* subsystemSentinelDrive;
    tap::arch::MilliTimeout changeVelocityTimer;
};

}  // namespace aruwsrc::control::sentinel::drive

#endif  // SENTINEL_RANDOM_DRIVE_COMMAND_HPP_

#endif
