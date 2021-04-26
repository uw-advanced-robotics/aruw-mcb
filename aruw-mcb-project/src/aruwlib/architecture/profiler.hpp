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

#ifndef __ARUW_PROFILER_HPP__
#define __ARUW_PROFILER_HPP__

#include <modm/container.hpp>

#define PROFILE(profiler, func, params) func params

#ifdef RUN_WITH_PROFILING
#undef PROFILE
#define PROFILE(profiler, func, params) \
    profiler.push(#func);               \
    func params;                        \
    profiler.pop()

#endif

namespace aruwlib
{
namespace arch
{
class Profiler
{
#ifdef RUN_WITH_PROFILING

public:
    void push(const char* profile);
    void pop();

    const modm::DynamicArray<const char*>& getAllProfiles() const { return profiles; }

    uint64_t getAvgTime(const char* profile);
    uint64_t getMinTime(const char* profile);
    uint64_t getMaxTime(const char* profile);
    void reset(const char* profile);

    struct MinMaxAvgStruct
    {
        uint64_t min = UINT32_MAX, max = 0, avg = 0;
        int totalValues = 0;
    };

private:
    modm::DynamicArray<modm::Pair<const char*, MinMaxAvgStruct>> times;
    modm::DynamicArray<const char*> profiles;
    modm::LinkedList<const char*> profileStack;
    modm::LinkedList<uint32_t> timeStack;

#else

public:
    void push(const char*){};
    void pop(){};

#endif

};  // class Profiler

}  // namespace arch

}  // namespace aruwlib

#endif