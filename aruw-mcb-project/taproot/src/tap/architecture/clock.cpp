/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)

#include "clock.hpp"

namespace tap
{
namespace arch
{
namespace clock
{
/**
 * Global static variable storing time for testing. It's value is returned from the `getTime*()`
 * functions and is set by `setTime()`. Note this is a static global variable accessible
 * from _any_ test, so you must assume that `getTimeMilliseconds()` value is undefined until
 * you set it.
 */
uint32_t currTimeMilliseconds = 0;

void setTime(uint32_t timeMilliseconds) { currTimeMilliseconds = timeMilliseconds; }

uint32_t getTimeMilliseconds() { return currTimeMilliseconds; }

uint32_t getTimeMicroseconds() { return currTimeMilliseconds * 1000; }
}  // namespace clock
}  // namespace arch
}  // namespace tap

#endif
