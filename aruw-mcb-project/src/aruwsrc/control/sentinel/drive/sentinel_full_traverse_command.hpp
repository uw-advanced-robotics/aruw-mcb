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
#ifndef SENTINEL_FULL_TRAVERSE_COMMAND_HPP_
#define SENTINEL_FULL_TRAVERSE_COMMAND_HPP_

#include "aruwlib/algorithms/ramp.hpp"
#include "aruwlib/architecture/clock.hpp"
#include "aruwlib/control/command.hpp"

#include "sentinel_drive_subsystem.hpp"

namespace aruwsrc::control::sentinel::drive
{
class SentinelDriveSubsystem;

class SentinelFullTraverseCommand : public aruwlib::control::Command
{
public:
    /**
     * @param[in] subsystem The sentinel chassis to drive.
     * @param[in] rampSpeed Rate of change of the sentinel when changing direction, in wheel RPM /
     *      ms.
     * @param[in] maxDesiredTraverseSpeed The rotational speed of the sentinel's wheels before
     *      gearing is applied, in RPM.
     * @param[in] turnaroundBuffer The distance from the end of the rail at which the sentinel will
     *      referse direction.
     * @param[in] railLength The length of the rail.
     * @param[in] sentinelLength The length of the sentinel.
     */
    explicit SentinelFullTraverseCommand(
        SentinelDriveSubsystem* subsystem,
        float rampSpeed,
        float maxDesiredTraverseSpeed,
        float turnaroundBuffer,
        float railLength,
        float sentinelLength);

    const char* getName() const override { return "sentinel full traverse"; }
    void initialize() override;
    void execute() override;
    void end(bool) override;
    bool isFinished() const override;

private:
    const float RAMP_SPEED;
    const float MAX_DESIRED_TRAVERSE_SPEED;
    const float TURNAROUND_BUFFER;
    const float RAIL_LENGTH;
    const float SENTINEL_LENGTH;

    uint32_t prevTime;

    aruwlib::algorithms::Ramp velocityTargetGenerator;

    SentinelDriveSubsystem* subsystemSentinelDrive;
};  // class SentinelFullTraverseCommand

}  // namespace aruwsrc::control::sentinel::drive

#endif  // SENTINEL_FULL_TRAVERSE_COMMAND_HPP_
#endif
