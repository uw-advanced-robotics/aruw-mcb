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

#include <cassert>

#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"

#include "aruwsrc/util_macros.hpp"

#include "sentinel_drive_subsystem.hpp"

namespace aruwsrc::control::sentinel::drive
{
/*
 * A command that causes the robot to move a random distance at a random RPM (always in the opposite
 * direction). Takes parameter speedFraction, which scales the speed of the random movement
 * accordingly.
 */
class SentinelDriveEvadeCommand : public tap::control::Command
{
public:
    static constexpr int32_t MIN_RPM = 5000;
    static constexpr int32_t MAX_RPM = 7000;
    static constexpr float LARGE_ARMOR_PLATE_WIDTH = 200.0f;
    static constexpr float MAX_TRAVERSE_DISTANCE = LARGE_ARMOR_PLATE_WIDTH + 300;
    static constexpr float TURNAROUND_BUFFER =
        0.2f * (SentinelDriveSubsystem::RAIL_LENGTH - SentinelDriveSubsystem::SENTINEL_LENGTH);

    /**
     * @param[in] sentinelDriveSubsystem The drive subsystem this Command is controlling.
     * @param[in] speedFraction A fraction that scales the speed of the random movement. This value
     * must be between [0, 1].
     */
    explicit SentinelDriveEvadeCommand(
        SentinelDriveSubsystem* sentinelDriveSubsystem,
        float speedFraction);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "sentinel random drive"; }

    /// @return The current distance to drive from the position when the direction changed, in
    /// millimeters.
    float getDistanceToDrive() const { return this->distanceToDrive; }

private:
    SentinelDriveSubsystem* sentinelDriveSubsystem;
    const float speedFactor;

    /// Position in millimeters where the sentinel was when it last changed direction
    float positionWhenDirectionChanged = 0;

    /// Distance to drive, in millimeters
    float distanceToDrive = 0;

    void reverseDirection(int32_t minDistance, int32_t maxDistance);

    void reverseDirectionIfCloseToEnd(float currentPosition);

    /// @return the minimum desired RPM as determined by the specified speedFactor passed in upon
    /// constructon
    inline int32_t getMinDesiredRpm() const { return round(MIN_RPM * speedFactor); }

    /// @return the minimum desired RPM as determined by the specified speedFactor passed in upon
    /// constructon
    inline int32_t getMaxDesiredRpm() const { return round(MAX_RPM * speedFactor); }

    /**
     * @param[in] currentPosition The current position of the sentinel chassis, in millimeters.
     * @return True if the sentinel has traveled distanceToDrive.
     */
    inline bool hasTraveledDistanceToDrive(float currentPosition) const
    {
        return abs(this->positionWhenDirectionChanged - currentPosition) >= this->distanceToDrive;
    }

    static uint32_t getRandomInteger()
    {
#ifndef PLATFORM_HOSTED
        if (modm::platform::RandomNumberGenerator::isReady())
        {
            return modm::platform::RandomNumberGenerator::getValue();
        }
        else
        {
            return 0;
        }
#else
        return 0;
#endif
    }

    /// @return a random integer within `[min, max)`. `min` must be <= `max`.
    static inline int32_t getRandomIntegerBetweenBounds(int32_t min, int32_t max)
    {
        assert(min <= max);

        uint32_t range = max - min;
        uint32_t randomValue = getRandomInteger();
        uint32_t randomValueWithinRange = randomValue % range;

        return static_cast<int32_t>(randomValueWithinRange) + min;
    }

    /**
     * @param[in] min Min chassis speed, in wheel RPM.
     * @param[in] max Max chassis speed, in wheel RPM.
     * @param[in] currentDesiredRpm The current desired chassis speed, in wheel RPM.
     *
     * @return a new random desired RPM between `[min, max)` that has the opposite sign of the
     * specified `currentDesiredRpm`.
     */
    static inline int32_t getNewDesiredRpm(int32_t min, int32_t max, float currentDesiredRpm)
    {
        int32_t randomValue = getRandomIntegerBetweenBounds(min, max);
        return copysign(randomValue, -currentDesiredRpm);
    }

    /**
     * @param[in] currentPosition The current position of the sentinel chassis, in millimeters.
     * @return true if the sentinel is near the start of the rail (as indicated by the
     * `currentPosition`).
     */
    static inline bool nearStartOfRail(float currentPosition)
    {
        return currentPosition <= TURNAROUND_BUFFER;
    }

    /**
     * @param[in] currentPosition The current position of the sentinel chassis, in millimeters.
     * @return true if the sentinel is near the end of the rail (as indicated by the
     * `currentPosition`).
     */
    static inline bool nearEndOfRail(float currentPosition)
    {
        static constexpr float RAIL_END_POSITION_WITH_TURNAROUND_BUFFER =
            SentinelDriveSubsystem::RAIL_LENGTH - SentinelDriveSubsystem::SENTINEL_LENGTH -
            TURNAROUND_BUFFER;

        return currentPosition >= RAIL_END_POSITION_WITH_TURNAROUND_BUFFER;
    }
};

}  // namespace aruwsrc::control::sentinel::drive

#endif  // SENTINEL_DRIVE_EVADE_COMMAND_HPP_
