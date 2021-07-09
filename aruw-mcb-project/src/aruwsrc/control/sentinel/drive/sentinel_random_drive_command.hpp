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

#include "aruwlib/architecture/timeout.hpp"
#include "aruwlib/control/command.hpp"

#include "sentinel_drive_subsystem.hpp"

namespace aruwsrc::control::sentinel::drive
{
class SentinelRandomDriveCommand : public aruwlib::control::Command
{
public:
    /**
     * @param[in] subsystem The sentinel chassis to drive.
     * @param[in] minRpm Minimum target RPM for random driving.
     * @param[in] maxRpm Maximum target RPM for random driving.
     * @param[in] changeTimerInterval Time between changing the target RPM.
     * @param[in] turnaroundBuffer The distance from the end of the rail at which the sentinel will
     *      referse direction.
     * @param[in] railLength Length of the rail the sentinel is driving on.
     * @param[in] sentinelLength Horizontal length of the sentienl.
     */
    explicit SentinelRandomDriveCommand(
        SentinelDriveSubsystem* subsystem,
        uint16_t minRpm,
        uint16_t maxRpm,
        uint16_t changeTimeInterval,
        float turnaroundBuffer,
        float railLength,
        float sentinelLength);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "sentinel random drive"; }

private:
    const int16_t MIN_RPM;
    const int16_t MAX_RPM;
    const int16_t CHANGE_TIME_INTERVAL;
    const float TURNAROUND_BUFFER;
    const float RAIL_LENGTH;
    const float SENTINEL_LENGTH;

    float currentRPM = 0;
    bool chosenNewRPM = false;

    SentinelDriveSubsystem* subsystemSentinelDrive;
    aruwlib::arch::MilliTimeout changeVelocityTimer;
};

}  // namespace aruwsrc::control::sentinel::drive

#endif  // SENTINEL_RANDOM_DRIVE_COMMAND_HPP_

#endif
