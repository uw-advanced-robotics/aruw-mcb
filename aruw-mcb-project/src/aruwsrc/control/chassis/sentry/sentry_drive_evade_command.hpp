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

#ifndef SENTRY_DRIVE_EVADE_COMMAND_HPP_
#define SENTRY_DRIVE_EVADE_COMMAND_HPP_

#include <cassert>

#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"

#include "sentry_drive_subsystem.hpp"

namespace aruwsrc::control::sentry::drive
{
/*
 * A command that causes the robot to move a random distance at a random RPM (always in the opposite
 * direction). Takes parameter speedFraction, which scales the speed of the random movement
 * accordingly.
 */
class SentryDriveEvadeCommand : public tap::control::Command
{
public:
    /// Minimum random drive speed, in RPM
    static constexpr int32_t MIN_RPM = 7'000;
    /// Maximum random drive speed, in RPM
    static constexpr int32_t MAX_RPM = 8'000;
    /// Width of a large armor plate, in mm. Used as the minimum traverse distance. A random
    /// distance between this and MAX_TRAVERSE_DISTANCE will be choosen by the sentry when it is
    /// not in the turnaround buffer.
    static constexpr float LARGE_ARMOR_PLATE_WIDTH = 200.0f;
    /// Max distance that the sentry will randomly travel when not in the turnaround buffer, in
    /// mm.
    static constexpr float MAX_TRAVERSE_DISTANCE = LARGE_ARMOR_PLATE_WIDTH + 300;
    /// Distance in mm from either side of the rail within which the sentry will turn around to
    /// avoid slamming into the sides. When turning around in the turnaround buffer, the sentry
    /// will travel past the centerpoint of the rail.
    static constexpr float TURNAROUND_BUFFER = 150.0f;

    /**
     * @param[in] sentryDriveSubsystem The drive subsystem this Command is controlling.
     * @param[in] speedFraction A fraction that scales the speed of the random movement. This value
     * must be between [0, 1].
     */
    explicit SentryDriveEvadeCommand(
        SentryDriveSubsystem* sentryDriveSubsystem,
        float speedFraction);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "sentry drive evade"; }

    /// @return The current distance to drive from the position when the direction changed, in
    /// millimeters.
    float getDistanceToDrive() const { return this->distanceToDrive; }

private:
    SentryDriveSubsystem* sentryDriveSubsystem;
    float speedFactor;

    /// Position in millimeters where the sentry was when it last changed direction
    float positionWhenDirectionChanged = 0;

    /// Distance to drive, in millimeters
    float distanceToDrive = 0;

    /**
     * Updates the sentry drive subsystem such that it drives in the opposite direction that it is
     * currently driving. The `distanceToDrive` and `sentryDriveSubsystem`'s `desiredRpm` will be
     * updated to some random values. The random distance choosen will be from `[minDistance,
     * maxDistance)` and the random speed will be between `speedFactor * [MIN_RPM, MAX_RPM)`.
     *
     * @param[in] minDistance Minimum random distance to drive, in millimeters.
     * @param[in] maxDistance Maximum random distance to drive, in millimeters.
     */
    void reverseDirectionForRandomDistance(int32_t minDistance, int32_t maxDistance);

    /**
     * Checks the distance from the end of the rail and reverses the direction of the sentry if it
     * is close to either end of the rail. Will set the `distanceToDrive` such that the sentry
     * will drive to at least the middle of the rail.
     *
     * @param[in] currentPosition The current position of the sentry along the rail, in
     * millimeters.
     */
    void reverseDirectionIfCloseToEnd(float currentPosition);

    /// @return the minimum desired RPM as determined by the specified speedFactor passed in upon
    /// constructon
    inline int32_t getMinDesiredRpm() const { return round(MIN_RPM * speedFactor); }

    /// @return the minimum desired RPM as determined by the specified speedFactor passed in upon
    /// constructon
    inline int32_t getMaxDesiredRpm() const { return round(MAX_RPM * speedFactor); }

    /**
     * @param[in] currentPosition The current position of the sentry chassis, in millimeters.
     * @return True if the sentry has traveled distanceToDrive.
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
};

}  // namespace aruwsrc::control::sentry::drive

#endif  // SENTRY_DRIVE_EVADE_COMMAND_HPP_
