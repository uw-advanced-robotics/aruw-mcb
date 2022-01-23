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

#ifndef CHASSIS_ORIENTATION_GETTER_INTERFACE_HPP_
#define CHASSIS_ORIENTATION_GETTER_INTERFACE_HPP_

namespace tap::control::odometry
{
/**
 * Object used to get chassis orientation relative to field x-axis. Positive
 * angles sweep from field x-axis to field y-axis (right-handed system).
 *
 * Returned angle is from sweeping from field positive x-axis to chassis forward
 * vector.
 *
 * @note Positive z-axis is straight upwards (opposite gravity). The orientation
 *  of the positive x and positive y axes are implementation-defined.
 *
 * Getting chassis orientation may fail as implementor chooses by returning
 * `false` to indicate either values are too stale or sensor went offline etc.
 * If return value is `false` odometry logic won't run that tick.
 */
class ChassisOrientationGetterInterface
{
public:
    /**
     * Get the chassis' "Yaw" in the world frame. Angle sweeps from positive
     * x-axis to positive y-axis. Positive z-axis is upwards.
     * @param[out] output destination for chassis orientation in radians.
     *      Range is in [0, 2PI).
     *      Value will be 0 if valid data unavailable
     * @return `true` if valid chassis orientation data was available,
     *      `false` otherwise.
     */
    virtual bool getChassisOrientation(float* output) = 0;
};

}  // namespace tap::control::odometry

#endif  // CHASSIS_ORIENTATION_GETTER_INTERFACE_HPP_
