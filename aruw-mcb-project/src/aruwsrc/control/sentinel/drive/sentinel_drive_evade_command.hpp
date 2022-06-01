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

#ifndef SENTINEL_DRIVE_EVADE_COMMAND_HPP_
#define SENTINEL_DRIVE_EVADE_COMMAND_HPP_

#include "aruwsrc/util_macros.hpp"

#if defined(ALL_SENTINELS)

#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"

#include "sentinel_drive_subsystem.hpp"

namespace aruwsrc::control::sentinel::drive
{
/*
 * A command that causes the robot to move a random distance
 * at a random RPM (always in the opposite direction). Takes parameter speedFraction,
 * which scales the speed of the random movement accordingly.
 */
class SentinelDriveEvadeCommand : public tap::control::Command
{
public:
    explicit SentinelDriveEvadeCommand(SentinelDriveSubsystem* subsystem, float speedFraction);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "sentinel random drive"; }

private:
    static const int16_t MIN_RPM = 5000;
    static const int16_t MAX_RPM = 7000;
    static const int16_t CHANGE_TIME_INTERVAL = 750;
    static constexpr float LARGE_ARMOR_PLATE_WIDTH = 200.0f;
    static constexpr float MAX_TRAVERSE_DISTANCE = LARGE_ARMOR_PLATE_WIDTH + 300;
    static constexpr float TURNAROUND_BUFFER = 0.2f * SentinelDriveSubsystem::RAIL_LENGTH;

    float currentRPM = 0;
    float positionWhenDirectionChanged = 0;
    int randDistance = 0;

    SentinelDriveSubsystem* sentinelDriveSubsystem;
    const float speedFactor;

    uint32_t portableRandom();
    void changeDirection(int minRPM, int maxRPM, int64_t minDist, int64_t maxDist);
    void setCurrentRPM(int min, int max);
    float getRandomVal(int64_t min, int64_t max);
};

}  // namespace aruwsrc::control::sentinel::drive

#endif

#endif  // SENTINEL_DRIVE_EVADE_COMMAND_HPP_
