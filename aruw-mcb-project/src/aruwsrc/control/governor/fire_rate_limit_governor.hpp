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

#ifndef FIRE_RATE_LIMIT_GOVERNOR_HPP_
#define FIRE_RATE_LIMIT_GOVERNOR_HPP_

#include "tap/control/governor/command_governor_interface.hpp"
#include <cmath>

#include "aruwsrc/control/agitator/fire_rate_timer_manager.hpp"

namespace aruwsrc::control::governor
{
enum class FireRateReadinessState
{
    READY_IGNORE_RATE_LIMITING = 0,
    READY_USE_RATE_LIMITING,
    NOT_READY,
};

/**
 * An interface that can be implemented to convey fire rate information to the
 * FireRateLimitGovernor.
 */
class FireRateManagerInterface
{
public:
    /// @return the fire rate period (time distance between launching projectiles)
    virtual uint32_t getFireRatePeriod() = 0;

    /// @return the readiness state of the
    virtual FireRateReadinessState getFireRateReadinessState() = 0;

protected:
    /**
     * Converts a rounds-per-second value (i.e., Hz) to a period in milliseconds.
     */
    static inline constexpr uint32_t rpsToPeriodMS(float rps)
    {
        if (rps == 0)
        {
            return UINT32_MAX;
        }
        else
        {
            return round(1000.0f / rps);
        }
    }
};

/**
 * A governor that allows a Command to run based on an internal timer and information from the
 * vision processor that dictates fire rate.
 *
 * Limits the frequency with which the underlying command is scheduled to be at most the last
 * "fire rate" suggestion provided by the Vision Coprocessor for this turret.
 *
 * If CV is disconnected, does not limit fire.
 */
class FireRateLimitGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    FireRateLimitGovernor(FireRateManagerInterface &fireRateManager)
        : fireRateManager(fireRateManager)
    {
    }

    void initialize() final
    {
        uint32_t fireRatePeriod = fireRateManager.getFireRatePeriod();

        fireRateTimerManager.setProjectileLaunched(fireRatePeriod);
    }

    bool isReady() final
    {
        auto readinessState = fireRateManager.getFireRateReadinessState();

        switch (readinessState)
        {
            case FireRateReadinessState::READY_IGNORE_RATE_LIMITING:
                return true;
            case FireRateReadinessState::READY_USE_RATE_LIMITING:
                return fireRateTimerManager.isReadyToLaunchProjectile();
            case FireRateReadinessState::NOT_READY:
                return false;
            default:
                return false;
        }
    }

    bool isFinished() final
    {
        // Never explicitly force the command to stop. We assume that it will fire at most one shot
        // then end by itself.
        return false;
    }

private:
    FireRateManagerInterface &fireRateManager;
    control::agitator::FireRateTimerManager fireRateTimerManager;
};
}  // namespace aruwsrc::control::governor

#endif  // FIRE_RATE_LIMIT_GOVERNOR_HPP_
