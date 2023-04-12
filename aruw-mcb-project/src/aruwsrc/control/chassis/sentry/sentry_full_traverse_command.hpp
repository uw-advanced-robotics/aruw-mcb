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

#ifndef SENTRY_FULL_TRAVERSE_COMMAND_HPP_
#define SENTRY_FULL_TRAVERSE_COMMAND_HPP_

#include "aruwsrc/util_macros.hpp"

#if defined(ALL_SENTRIES)

#include "tap/algorithms/ramp.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/control/command.hpp"

#include "sentry_drive_subsystem.hpp"

namespace aruwsrc::control::sentry::drive
{
class SentryDriveSubsystem;

class SentryFullTraverseCommand : public tap::control::Command
{
public:
    explicit SentryFullTraverseCommand(SentryDriveSubsystem* subsystem);

    const char* getName() const override { return "sentry full traverse"; }
    void initialize() override;
    void execute() override;
    void end(bool) override;
    bool isFinished() const override;

private:
    /**
     * Rate of change of the sentry when changing direction, in wheel RPM / ms
     */
    static constexpr float RAMP_SPEED = 10.0f;

    /**
     * The rotational speed of the sentry's wheels before gearing is applied, in RPM.
     */
    static constexpr float MAX_DESIRED_TRAVERSE_SPEED = 3000.0f;

    /**
     * The distance from the end of the rail at which the sentry will referse direction.
     */
    static constexpr float TURNAROUND_BUFFER = 0.2f * SentryDriveSubsystem::RAIL_LENGTH;

    uint32_t prevTime;

    tap::algorithms::Ramp velocityTargetGenerator;

    SentryDriveSubsystem* subsystemSentryDrive;
};  // class SentryFullTraverseCommand

}  // namespace aruwsrc::control::sentry::drive

#endif

#endif  // SENTRY_FULL_TRAVERSE_COMMAND_HPP_
