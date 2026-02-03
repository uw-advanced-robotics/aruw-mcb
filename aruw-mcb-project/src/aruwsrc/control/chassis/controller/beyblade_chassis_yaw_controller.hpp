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

#ifndef BEYBLADE_CHASSIS_YAW_CONTROLLER_HPP_
#define BEYBLADE_CHASSIS_YAW_CONTROLLER_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/ramp.hpp"

#include "chassis_yaw_controller_interface.hpp"

namespace aruwsrc::control::chassis::controller
{
class BeybladeChassisYawController : public ChassisYawControllerInterface
{
public:
    BeybladeChassisYawController() {}

    void initialize() override
    {
#ifdef ENV_UNIT_TESTS
        rotationDirection = 1;
#else
        rotationDirection = (rand() - RAND_MAX / 2) < 0 ? -1 : 1;
#endif
    }

    float runYawController(const float maxSpeed) override
    {
        return rotationDirection * config.beybladeRotationalSpeedFractionOfMax * maxWheelSpeed *
               rotationMultiplier;
    }

private:
    int8_t rotationDirection;
};  // class ChassisYawControllerInterface

}  // namespace aruwsrc::control::chassis::controller

#endif  // BEYBLADE_CHASSIS_YAW_CONTROLLER_HPP_
