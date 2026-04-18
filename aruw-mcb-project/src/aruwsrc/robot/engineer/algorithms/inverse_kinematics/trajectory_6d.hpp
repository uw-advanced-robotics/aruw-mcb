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

#include "tap/algorithms/transforms/transform.hpp"

#include "modm/math/geometry/quaternion.hpp"

namespace aruwsrc::engineer::algorithms::inverse_kinematics
{
/// TODO: this shouldn't live here
modm::Quaternion<float> quaternionFromRPY(float r, float p, float y)
{
    float cr = cosf(r / 2);
    float sr = sinf(r / 2);
    float cp = cosf(p / 2);
    float sp = sinf(p / 2);
    float cy = cosf(y / 2);
    float sy = sinf(y / 2);

    return modm::Quaternion<float>(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy);
}

/// TODO: this shouldn't live here
template <class T>
modm::Quaternion<T> slerp(modm::Quaternion<T> q0, modm::Quaternion<T> q1, float t)
{
    float theta = acosf(q0.w * q1.w + q0.x * q1.x + q0.y * q1.y + q0.z * q1.z);

    return q0 * (sinf((1 - t) * theta) / sinf(theta)) + q1 * (sinf(t * theta) / sinf(theta))
}

struct Waypoint
{
    tap::algorithms::transforms::Position position;
    modm::Quaternion<float> rotation;
    float time;
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

        return tap::algorithms::transforms::Transform(
            tap::algorithms::transforms::Position::interpolate(
                waypoints[i].position,
                waypoints[i + 1].position,
                t),
            slerp(waypoints[i].rotation, waypoints[i + 1].rotation, t))
    }
};

}  // namespace aruwsrc::engineer::algorithms::inverse_kinematics
#endif  // TRAJECTORY_6D_HPP_
