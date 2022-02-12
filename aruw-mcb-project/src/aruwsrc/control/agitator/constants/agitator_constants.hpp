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
#include "tap/algorithms/smooth_pid.hpp"

#ifndef AGITATOR_CONSTANTS_HPP_
#define AGITATOR_CONSTANTS_HPP_

#if defined(ALL_SOLDIERS)
#include "soldier_agitator_constants.hpp"
#elif defined(TARGET_SENTINEL)
#include "sentinel_agitator_constants.hpp"
#elif defined(TARGET_HERO)
#include "hero_agitator_constants.hpp"
#endif
namespace aruwsrc::control::agitator::constants{
    static constexpr tap::algorithms::SmoothPidConfig PID_17MM = {
        .kp = 300'000.0f,
        .ki = 0.0f,
        .kd = 50.0f,
        .maxICumulative = 0.0f,
        .maxOutput =  16000.0f
    };
}
#endif
