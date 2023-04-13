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

#ifndef SENTRY_AUTO_DRIVE_CONSTANTS_HPP_
#define SENTRY_AUTO_DRIVE_CONSTANTS_HPP_

#include "modm/math/filter/pid.hpp"

namespace aruwsrc::chassis
{

static constexpr float AUTONAV_Kp = 1.0f;
static constexpr float AUTONAV_Ki = 0.0f;
static constexpr float AUTONAV_Kd = 0.0f;
static constexpr float AUTONAV_MaxIntegralErrorSum = 0.0f;
static constexpr float AUTONAV_MaxOutput = 16'384.0f;
static constexpr float AUTONAV_FeedForwardConstant = 0.0f;

}  // namespace aruwsrc::chassis

#endif  // SENTRY_AUTO_DRIVE_CONSTANTS_HPP_
