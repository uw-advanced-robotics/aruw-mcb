/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BALSTD_CHASSIS_CONSTANTS_HPP_
#define BALSTD_CHASSIS_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/motor/tmotor_ak80_9_encoder.hpp"

#include "balstd_leg.hpp"

namespace aruwsrc::control::balstd
{
static constexpr float OUTER_HARD_STOP = modm::toRadian(-20);
static constexpr float OUTER_SOFT_STOP = modm::toRadian(-15);

static constexpr float INNER_SOFT_STOP = modm::toRadian(90);

static constexpr int32_t FRONT_HIP_MOTOR_HOME = static_cast<int32_t>(
    -OUTER_HARD_STOP / M_TWOPI * aruwsrc::control::motor::Tmotor_AK809Encoder::ENC_RESOLUTION * 9);
static constexpr int32_t BACK_HIP_MOTOR_HOME = static_cast<int32_t>(
    (OUTER_HARD_STOP - M_PI) / M_TWOPI *
    aruwsrc::control::motor::Tmotor_AK809Encoder::ENC_RESOLUTION * 9);

BalstdLegConfig LEG_CONFIG{
    .upperLinkLength = 0.15,
    .lowerLinkLength = 0.25,
    .fixedLinkLength = 0.108,
    .frontHipOuterLimit = OUTER_SOFT_STOP,
    .frontHipInnerLimit = INNER_SOFT_STOP,
    .backHipOuterLimit = M_PI - OUTER_SOFT_STOP,
    .backHipInnerLimit = INNER_SOFT_STOP,
};

}  // namespace aruwsrc::control::balstd

#endif  // BALSTD_CHASSIS_CONSTANTS_HPP_
