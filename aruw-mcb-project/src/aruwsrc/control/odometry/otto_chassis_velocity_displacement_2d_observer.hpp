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

#ifndef OTTO_CHASSIS_VELOCITY_DISPLACEMENT_2D_OBSERVER_HPP_
#define OTTO_CHASSIS_VELOCITY_DISPLACEMENT_2D_OBSERVER_HPP_

#include <cstdint>

#include "tap/algorithms/odometry/chassis_displacement_observer_interface.hpp"

#include "modm/math/geometry/vector.hpp"
#include "modm/math/geometry/location_2d.hpp"

// Forward declarations
namespace aruwsrc::chassis
{
class ChassisSubsystem;
}

namespace aruwsrc::control::odometry
{
/**
 * @brief Class that gets 2D chassis displacement by integrating chassis velocity.
 *
 * For use in the Otto system. See parent class for more details.
 *
 * This class dumbs things down to only two dimensions for use with the 2D odometry
 * system.
 * 
 * Update should be called frequently for best results.
 */
class OttoChassisVelocityDisplacement2DObserver
    : public tap::algorithms::odometry::ChassisDisplacementObserverInterface
{
public:
    /**
     * @param[in] pointer to an aruwsrc ChassisSubsystem. Used to get chassis relative velocity
     */
    OttoChassisVelocityDisplacement2DObserver(aruwsrc::chassis::ChassisSubsystem* chassis);

    /**
     * Update the observed displacement. Call frequently
     */
    void update();

    /**
     * Get absolute chassis displacement in chassis frame in meters since some arbitrary point
     * in time.
     * 
     * @see ChassisDisplacementObserverInterface for more details
     *
     * @param[out] displacement
     *
     * @return `true` if valid chassis velocity was available. `false` otherwise.
     */
    bool getChassisDisplacement(modm::Vector<float, 3>* displacement) const final;

private:
    aruwsrc::chassis::ChassisSubsystem* chassis;
    modm::Location2D<float> absoluteDisplacement;
    // Previous time at which motors are all online. 0 iff motors were offline last update.
    uint32_t prevTime = 0;
    // Stores whether or not stored displacement is valid. `True` after first good update.
    bool dataValid;
};

}  // namespace aruwsrc::control::odometry

#endif  // OTTO_CHASSIS_VELOCITY_DISPLACEMENT_2D_OBSERVER_HPP_
