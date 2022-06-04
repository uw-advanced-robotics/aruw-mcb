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

#ifndef SENTINEL_DRIVE_TO_SIDE_COMMAND_HPP_
#define SENTINEL_DRIVE_TO_SIDE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "sentinel_drive_subsystem.hpp"

namespace aruwsrc::control::sentinel::drive
{
/**
 * Command that takes command of the sentinel chassis and moves the sentinel to one of the rail
 * sides.
 */
class SentinelDriveToSideCommand : public tap::control::Command
{
public:
    /// The fraction of the traversable rail length which is considered to be "near" the end of the
    /// rail multiplied by the traversable rail length.
    static constexpr float RAIL_LENGTH_END_THRESHOLD =
        0.05f * (SentinelDriveSubsystem::RAIL_LENGTH - SentinelDriveSubsystem::SENTINEL_LENGTH);

    enum class SentinelRailSide : uint8_t
    {
        CLOSE_RAIL,  ///< The rail side at 0 mm as defined by sentinel odometry
        FAR_RAIL,  ///< The rail side at SentinelDriveSubsystem::RAIL_LENGTH as defined by sentinel
                   ///< odometry
    };

    SentinelDriveToSideCommand(
        SentinelDriveSubsystem &sentinelChassis,
        SentinelRailSide railSide,
        float movementSpeedRpm);

    const char *getName() const override { return "Sentinel drive to side"; }
    bool isReady() override;
    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;

private:
    SentinelDriveSubsystem &sentinelChassis;

    const SentinelRailSide railSide;
    const float movementSpeedRpm;

    /**
     * @param[in] railSide The side of the rail that the sentinel should be near.
     * @param[in] sentinelPosition The current position of the sentinel along the rail.
     * @return true if the sentinel is close enough to the end of the rail that it is considered to
     * be at the end of the rail.
     */
    static inline bool withinRailEnd(SentinelRailSide railSide, float sentinelPosition)
    {
        switch (railSide)
        {
            case SentinelRailSide::CLOSE_RAIL:
                return sentinelPosition <= RAIL_LENGTH_END_THRESHOLD;
            case SentinelRailSide::FAR_RAIL:
                return sentinelPosition >= SentinelDriveSubsystem::RAIL_LENGTH -
                                               SentinelDriveSubsystem::SENTINEL_LENGTH -
                                               RAIL_LENGTH_END_THRESHOLD;
            default:
                return false;
        }
    }
};
}  // namespace aruwsrc::control::sentinel::drive

#endif  //  SENTINEL_DRIVE_TO_SIDE_COMMAND_HPP_
