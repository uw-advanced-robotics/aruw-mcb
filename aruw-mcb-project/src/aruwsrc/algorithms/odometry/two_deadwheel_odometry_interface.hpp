/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
        aruwsrc::virtualMCB::VirtualDjiMotor* perpendicularWheel,
        aruwsrc::virtualMCB::VirtualDjiMotor* parallelWheel);

    float getParallelMotorRPM() const;
    float getPerpendicularRPM() const;

private:
    aruwsrc::virtualMCB::VirtualDjiMotor* perpendicularWheel;
    aruwsrc::virtualMCB::VirtualDjiMotor* parallelWheel;
};

}  // namespace aruwsrc::algorithms::odometry

#endif  // OTTO_CHASSIS_WORLD_YAW_OBSERVER_HPP_