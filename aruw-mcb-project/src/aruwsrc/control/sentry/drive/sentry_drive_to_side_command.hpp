/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SENTRY_DRIVE_TO_SIDE_COMMAND_HPP_
#define SENTRY_DRIVE_TO_SIDE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "sentry_drive_subsystem.hpp"

namespace aruwsrc::control::sentry::drive
{
/**
 * Command that drives the sentry chassis to a specified side of the rail.
 */
class SentryDriveToSideCommand : public tap::control::Command
{
public:
    /// The fraction of the traversable rail length which is considered to be "near" the end of the
    /// rail multiplied by the traversable rail length.
    static constexpr float RAIL_LENGTH_END_THRESHOLD =
        0.05f * (SentryDriveSubsystem::RAIL_LENGTH - SentryDriveSubsystem::SENTRY_LENGTH);

    enum class SentryRailSide : uint8_t
    {
        CLOSE_RAIL,  ///< The rail side at 0 mm as defined by sentry odometry
        FAR_RAIL,    ///< The rail side at SentryDriveSubsystem::RAIL_LENGTH as defined by sentry
                     ///< odometry
    };

    SentryDriveToSideCommand(
        SentryDriveSubsystem &sentryChassis,
        SentryRailSide railSide,
        float movementSpeedRpm);

    const char *getName() const override { return "Sentry drive to side"; }
    bool isReady() override;
    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;

private:
    SentryDriveSubsystem &sentryChassis;

    const SentryRailSide railSide;
    const float movementSpeedRpm;

    /**
     * @param[in] railSide The side of the rail that the sentry should be near.
     * @param[in] sentryPosition The current position of the sentry along the rail.
     * @return true if the sentry is close enough to the end of the rail that it is considered to
     * be at the end of the rail.
     */
    static inline bool withinRailEnd(SentryRailSide railSide, float sentryPosition)
    {
        switch (railSide)
        {
            case SentryRailSide::CLOSE_RAIL:
                return sentryPosition <= RAIL_LENGTH_END_THRESHOLD;
            case SentryRailSide::FAR_RAIL:
                return sentryPosition >= SentryDriveSubsystem::RAIL_LENGTH -
                                             SentryDriveSubsystem::SENTRY_LENGTH -
                                             RAIL_LENGTH_END_THRESHOLD;
            default:
                return false;
        }
    }
};
}  // namespace aruwsrc::control::sentry::drive

#endif  //  SENTRY_DRIVE_TO_SIDE_COMMAND_HPP_
