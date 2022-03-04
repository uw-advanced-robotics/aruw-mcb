/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "profiler.hpp"

#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "../algorithms/math_user_utils.hpp"

#include "clock.hpp"

namespace tap::arch
{
Profiler::Profiler(tap::Drivers *drivers) : drivers(drivers) {}

std::size_t Profiler::push(const char *profile)
{
    auto index = elementNameToIndexMap.find(profile);
    if (index != elementNameToIndexMap.end())
    {
        ProfilerData *data = &profiledElements[index->second];
        data->prevPushedTime = clock::getTimeMicroseconds();
        return index->second;
    }
    else if (!profiledElements.isFull())
    {
        profiledElements.append(ProfilerData(profile));
        std::size_t key = profiledElements.getSize() - 1;
        elementNameToIndexMap[profile] = key;
        profiledElements[key].prevPushedTime = clock::getTimeMicroseconds();
        return key;
    }
    else
    {
        RAISE_ERROR(drivers, "profiler full, no more additional profiling data allowed");
        return profiledElements.getSize();
    }
}

void Profiler::pop(std::size_t key)
{
    if (key >= profiledElements.getSize())
    {
        RAISE_ERROR(drivers, "attempting to pop profile, but never pushed");
    }
    else
    {
        ProfilerData *data = &profiledElements[key];
        uint32_t dt = clock::getTimeMicroseconds() - data->prevPushedTime;
        data->max = std::max(dt, data->max);
        data->min = std::min(dt, data->min);
        data->avg = algorithms::lowPassFilter(data->avg, dt, AVG_LOW_PASS_ALPHA);
    }
}

}  // namespace tap::arch
