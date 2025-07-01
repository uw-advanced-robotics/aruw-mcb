/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BALSTD_CHASSIS_STATE_HPP_
#define BALSTD_CHASSIS_STATE_HPP_

#include "tap/algorithms/transforms/transform.hpp"

#include "balstd_leg.hpp"

namespace aruwsrc::balstd::chassis
{
struct BalstdChassisState
{
    BalstdLegState leftLegState, rightLegState, virtualLegState;
    float roll, rollVel, pitch, pitchVel, yaw, yawVel;
    float height;
    float virtualPendTheta, virtualPendThetaDot;
    float virtualWheelPos, virtualWheelVel;  // x pos/vel of the 2d robot model's wheel
};

const BalstdChassisState ZERO_STATE{
    .leftLegState = BalstdLegState(),
    .rightLegState = BalstdLegState(),
    .virtualLegState = BalstdLegState(),
    .roll = 0,
    .rollVel = 0,
    .pitch = 0,
    .pitchVel = 0,
    .yaw = 0,
    .yawVel = 0,
    .height = 0,
    .virtualPendTheta = 0,
    .virtualPendThetaDot = 0,
    .virtualWheelPos = 0,
    .virtualWheelVel = 0,
};

}  // namespace aruwsrc::balstd::chassis

#endif  // BALSTD_CHASSIS_STATE_HPP_