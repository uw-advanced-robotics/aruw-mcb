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

#ifndef CHASSIS_YAW_CONTROLLER_INTERFACE_HPP_
#define CHASSIS_YAW_CONTROLLER_INTERFACE_HPP_

namespace aruwsrc::control::chassis::controller
{
class ChassisYawControllerInterface
{
public:
    virtual void initialize() {}

    /**
     * @param maxVel Maximum yaw velocity as determined by the power limiting system
     * @return Desired yaw velocity
     */
    virtual float runYawController(const float maxSpeed) = 0;

    virtual bool isFinished() { return false; }
};  // class ChassisYawControllerInterface

}  // namespace aruwsrc::control::chassis::controller

#endif  // CHASSIS_YAW_CONTROLLER_INTERFACE_HPP_
