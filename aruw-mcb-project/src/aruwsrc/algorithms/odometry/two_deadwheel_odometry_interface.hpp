/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TWO_DEADWHEEL_ODOMETRY_INTERFACE_HPP_
#define TWO_DEADWHEEL_ODOMETRY_INTERFACE_HPP_

#include <aruwsrc/communication/mcb-lite/motor/virtual_dji_motor.hpp>

namespace aruwsrc::algorithms::odometry
{
class TwoDeadwheelOdometryInterface
{
public:
    TwoDeadwheelOdometryInterface(
        aruwsrc::virtualMCB::VirtualDjiMotor* parallelWheel,
        aruwsrc::virtualMCB::VirtualDjiMotor* perpendicularWheel,
        const float wheelRadius);

    const float WHEEL_RADIUS;

    /// Get RPM of odom wheel oriented such that it rolls on the tangent line to the chassis
    float getParallelMotorRPM() const;

    /// Get RPM of odom wheel oriented such that it rolls on the line perpendicular to the chassis
    float getPerpendicularRPM() const;

    inline float rpmToMetersPerSecond(float rpm) const { return rpm / 60 * M_TWOPI * WHEEL_RADIUS; }

private:
    /// Parallel wheel is oriented such that it rolls on the tangent line to the chassis
    aruwsrc::virtualMCB::VirtualDjiMotor* parallelWheel;
    /// Perpendicular wheel is oriented such that it rolls on the line perpendicular to the chassis
    aruwsrc::virtualMCB::VirtualDjiMotor* perpendicularWheel;
};

}  // namespace aruwsrc::algorithms::odometry

#endif  // OTTO_CHASSIS_WORLD_YAW_OBSERVER_HPP_