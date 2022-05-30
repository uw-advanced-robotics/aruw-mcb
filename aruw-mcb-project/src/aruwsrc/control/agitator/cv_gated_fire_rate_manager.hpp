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

#ifndef CV_GATED_FIRE_RATE_MANAGER_HPP_
#define CV_GATED_FIRE_RATE_MANAGER_HPP_

#include "aruwsrc/control/governor/cv_on_target_governor.hpp"

#include "fire_rate_manager.hpp"

namespace aruwsrc::control::agitator
{
/**
 * An object that extends the fire rate manager but utilizes a CvOnTargetGovernor to determine
 * whether or not to apply fire rate limiting.
 */
class CvGatedFireRateManager : public FireRateManager
{
public:
    CvGatedFireRateManager(governor::CvOnTargetGovernor &cvOnTargetGovernor)
        : cvOnTargetGovernor(cvOnTargetGovernor)
    {
    }

    /**
     * Returns `READY_IGNORE_RATE_LIMITING` if CV is online (as determined  by the
     * `CvOnlineGovernor`)
     *
     * @see FireRateManager::getFireRateReadinessState
     */
    inline control::governor::FireRateReadinessState getFireRateReadinessState() override
    {
        if (cvOnTargetGovernor.isGovernorGating())
        {
            return governor::FireRateReadinessState::READY_IGNORE_RATE_LIMITING;
        }

        return FireRateManager::getFireRateReadinessState();
    }

private:
    governor::CvOnTargetGovernor &cvOnTargetGovernor;
    float fireRate = 0;
};
}  // namespace aruwsrc::control::agitator

#endif  // CV_GATED_FIRE_RATE_MANAGER_HPP_
