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

#ifndef SETPOINT_SCANNER_HPP_
#define SETPOINT_SCANNER_HPP_

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"

namespace aruwsrc::control::turret::cv
{
/**
 * A simple class that bounces a setpoint back and forth between two bounds.
 *
 * Intended to be used for the Sentinel turret scanning logic.
 *
 * Takes the current setpoint from the user (to account for potential setpoint changes
 * when this class isn't the only thing controlling the setpoint). Should only be used
 * with a single setpoint.
 */
class SetpointScanner
{
public:
    /**
     * Create a SetpointScanner
     *
     * @param lowerBound the lower bound to bounce the setpoint within
     * @param upperBound the upper bound to bounce the setpoint within
     * @param delta the step to update the setpoint by every time scan() is called
     */
    SetpointScanner(float lowerBound, float upperBound, float delta)
        : lowerBound(lowerBound),
          upperBound(upperBound),
          delta(delta),
          scanningPositive(true)
    {
        assert(lowerBound <= upperBound);
    }

    /**
     * Update the setpoint by the delta and return the new setpoint.
     *
     * @param setpoint the current setpoint
     * @return the setpoint after being updated
     */
    inline float scan(float setpoint)
    {
        if (setpoint >= upperBound)
        {
            scanningPositive = false;
        }
        else if (setpoint <= lowerBound)
        {
            scanningPositive = true;
        }

        if (scanningPositive)
        {
            setpoint += delta;
        }
        else
        {
            setpoint -= delta;
        }

        // Bound value between upper and lower bounds
        return tap::algorithms::limitVal(setpoint, lowerBound, upperBound);
    }

private:
    const float lowerBound;
    const float upperBound;
    const float delta;
    bool scanningPositive;
};

}  // namespace aruwsrc::control::turret::cv

#endif  // SETPOINT_SCANNER_HPP_
