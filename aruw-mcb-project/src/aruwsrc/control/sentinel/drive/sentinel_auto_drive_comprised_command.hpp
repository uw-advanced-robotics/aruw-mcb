/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SENTINEL_AUTO_DRIVE_COMPRISED_COMMAND_HPP_
#define SENTINEL_AUTO_DRIVE_COMPRISED_COMMAND_HPP_

#include "tap/architecture/timeout.hpp"
#include "tap/control/comprised_command.hpp"

#include "aruwsrc/util_macros.hpp"

#include "sentinel_drive_evade_command.hpp"
#include "sentinel_drive_to_side_command.hpp"
#include "sentinel_full_traverse_command.hpp"

namespace aruwsrc::control::sentinel::drive
{
class SentinelDriveSubsystem;
/**
 * A command that alternates between base sentinel drive commands depending on the
 * damage it is currently engaging and **eventually** based on which targets are being
 * engaged
 */
class SentinelAutoDriveComprisedCommand : public tap::control::ComprisedCommand
{
public:
    SentinelAutoDriveComprisedCommand(
        aruwsrc::Drivers *drivers,
        SentinelDriveSubsystem *sentinelChassis);

    const char *getName() const override { return "sentinel random drive"; }
    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;

private:
    /// Threshold in damage per second above which the robot will enter aggressive drive mode.
    static constexpr float AGGRESSIVE_EVADE_DPS_THRESHOLD = 5;
    /// Minimum time in milliseconds spent aggressively driving when aggressive drive mode is
    /// entered.
    static constexpr uint32_t MIN_TIME_SPENT_AGGRESSIVELY_EVADING = 5'000;
    /// Speed in wheel RPM at which the sentinel will drive to the right side of the rail.
    static constexpr float MOVE_TO_RIGHT_DRIVE_SPEED_RPM = 3'000;

    aruwsrc::Drivers *drivers;
    tap::arch::MilliTimeout aggressiveEvadeTimer;
    SentinelDriveEvadeCommand aggressiveEvadeCommand;
    SentinelDriveEvadeCommand passiveEvadeCommand;
    SentinelDriveToSideCommand moveToFarRightCommand;
};
}  // namespace aruwsrc::control::sentinel::drive

#endif  // SENTINEL_AUTO_DRIVE_COMPRISED_COMMAND_HPP_
