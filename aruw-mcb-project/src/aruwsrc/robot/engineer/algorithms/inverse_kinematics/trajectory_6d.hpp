/*
 * Copyright (c) 2025-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef TRAJECTORY_6D_HPP_
#define TRAJECTORY_6D_HPP_

#include <array>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/transforms/transform.hpp"

#include "modm/math/geometry/quaternion.hpp"

namespace aruwsrc::engineer::algorithms::inverse_kinematics
{
struct Waypoint
{
    tap::algorithms::transforms::Transform pose;
    float time;  // seconds
};

template <size_t LENGTH>
struct Trajectory6D
{
    std::array<Waypoint, LENGTH> waypoints;

    tap::algorithms::transforms::Transform atTime(float time) const
    {
        size_t i = 0;
        while (i < LENGTH - 1 && waypoints[i + 1].time < time) i++;

        float t = (time - waypoints[i].time) / (waypoints[i + 1].time - waypoints[i].time);

        return tap::algorithms::transforms::Transform::interpolate(
            waypoints[i].pose,
            waypoints[i + 1].pose,
            t);
    }
};

}  // namespace aruwsrc::engineer::algorithms::inverse_kinematics
#endif  // TRAJECTORY_6D_HPP_
