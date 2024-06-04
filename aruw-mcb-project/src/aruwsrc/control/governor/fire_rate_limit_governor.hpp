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

#include "aruwsrc/control/agitator/fire_rate_reselection_manager_interface.hpp"
#include "aruwsrc/control/agitator/fire_rate_timer.hpp"

namespace aruwsrc::control::governor
{
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
    /**
     * @param[in] fireRateReselectionManager Object that reports a specified projectile rate of fire
     * and a projectile rate of fire readiness state that this governor uses to determine if a
     * projectile should be launched.
     */
    FireRateLimitGovernor(agitator::FireRateReselectionManagerInterface &fireRateReselectionManager)
        : fireRateReselectionManager(fireRateReselectionManager)
    {
    }

    void onGovernedCommandInitialized() final { fireRateTimer.registerNewLaunchedProjectile(); }

    bool isReady() final
    {
        uint32_t fireRatePeriod = fireRateReselectionManager.getFireRatePeriod();
        fireRateTimer.setProjectileLaunchPeriod(fireRatePeriod);

        auto readinessState = fireRateReselectionManager.getFireRateReadinessState();
        switch (readinessState)
        {
            case agitator::FireRateReadinessState::READY_IGNORE_RATE_LIMITING:
                return true;
            case agitator::FireRateReadinessState::READY_USE_RATE_LIMITING:
                return fireRateTimer.isReadyToLaunchProjectile();
            case agitator::FireRateReadinessState::NOT_READY:
                return false;
            default:
                return false;
        }
    }

    bool isFinished() final
    {
        // Never explicitly force the command to stop. We assume that it will fire at most one shot
        // then end by itself.
        return isReady();
    }

private:
    agitator::FireRateReselectionManagerInterface &fireRateReselectionManager;
    control::agitator::FireRateTimer fireRateTimer;
};
}  // namespace aruwsrc::control::governor

#endif  // FIRE_RATE_LIMIT_GOVERNOR_HPP_
