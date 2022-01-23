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

#ifndef OTTO_CHASSIS_VELOCITY_GETTER_HPP_
#define OTTO_CHASSIS_VELOCITY_GETTER_HPP_

#include "tap/control/odometry/chassis_displacement_getter_interface.hpp"

// Forward declarations
namespace aruwsrc::chassis
{
class ChassisSubsystem;
}

namespace aruwsrc::control::odometry
{
/**
 * @brief Class that gets chassis displacement by integrating chassis velocity.
 *
 * For use in the Otto system. See parent class for more details.
 */
class OttoChassisVelocityDisplacementGetter
    : public tap::control::odometry::ChassisDisplacementGetterInterface
{
public:
    /**
     * @param[in] pointer to an aruwsrc ChassisSubsystem. Used to get chassis relative velocity
     */
    OttoChassisVelocityDisplacementGetter(aruwsrc::chassis::ChassisSubsystem* chassis);

    /**
     * Get chassis displacement in chassis frame. Positive x, y, z, is chassis forward, left, and up
     * respectively.
     * @param[out] x destination for x velocity, 0 if valid data unavailable
     * @param[out] y destination for y velocity, 0 if valid data unavailable
     * @return `true` if valid chassis velocity was available. `false` otherwise.
     */
    bool getChassisDisplacement(float* x, float* y) final;

private:
    aruwsrc::chassis::ChassisSubsystem* chassis;
};

}  // namespace aruwsrc::control::odometry

#endif  // OTTO_CHASSIS_VELOCITY_GETTER_HPP_
