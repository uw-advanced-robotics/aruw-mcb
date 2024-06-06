/*
 * Copyright (c) 2021-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef MOVING_AVERAGE_HPP_
#define MOVING_AVERAGE_HPP_

#include <queue>

/**
 * Simple moving average filter.
 */
class MovingAverage
{
public:
    MovingAverage(size_t windowSize) : windowSize(windowSize) {}

    void update(float val)
    {
        sum += val;
        vals.push(val);
        if (vals.size() > windowSize)
        {
            sum -= vals.front();
            vals.pop();
        }
    }

    inline float getValue() const { return sum / windowSize; };

    const size_t windowSize;

private:
    std::queue<float> vals;
    float sum = 0;
};

#endif  // MOVING_AVERAGE_HPP_