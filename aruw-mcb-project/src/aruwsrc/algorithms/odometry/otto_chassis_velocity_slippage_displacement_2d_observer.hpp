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

#ifndef OTTO_CHASSIS_VELOCITY_SLIPPAGE_DISPLACEMENT_2D_OBSERVER_HPP_
#define OTTO_CHASSIS_VELOCITY_SLIPPAGE_DISPLACEMENT_2D_OBSERVER_HPP_

#include <cstdint>

#include "tap/algorithms/odometry/chassis_displacement_observer_interface.hpp"

#include "modm/math/geometry/location_2d.hpp"
#include "modm/math/geometry/vector.hpp"

// Forward declarations
namespace tap::control::chassis
{
class ChassisSubsystemInterface;
}

namespace aruwsrc::algorithms::odometry
{
/**
 * @brief Class that gets 2D chassis displacement by integrating chassis velocity and accounts
 * for slippage by limiting the max acceleration the velocity can experience.
 *
 * Update should be called frequently for best results.
 *
 * @note max acceleration threshold can be found experimentally by recording the maximum
 * acceleration the IMU reports when the chassis tries to fully accelerate. There are
 * limits to this as realistically the max acceleration in different directions could be different
 * depending on chassis setup and motor layout. This class does not account for that.
 *
 * This class also does not account for high angular velocities causing large sudden (but also
 * valid) changes in chassis frame velocity.
 */
class OttoChassisVelocitySlippageDisplacement2DObserver
    : public tap::algorithms::odometry::ChassisDisplacementObserverInterface
{
public:
    /**
     * @param[in] pointer to an aruwsrc ChassisSubsystem. Used to get chassis relative velocity
     */
    OttoChassisVelocitySlippageDisplacement2DObserver(
        tap::control::chassis::ChassisSubsystemInterface* chassis);

    /**
     * Update the observed displacement. Call frequently
     */
    void update();

    /**
     * Get absolute chassis displacement in chassis frame in meters since some arbitrary point
     * in time.
     *
     * No intuitive physical meaning when interpreted on it's own. Instead it is expected that
     * this value is sampled frequently and that differences between subsequent samples are used
     * for displacement in some consistent coordinate frame.
     *
     * @see ChassisDisplacementObserverInterface for more details
     *
     * @param[out] velocity
     * @param[out] displacement
     *
     * @return `true` if valid chassis velocity was available. `false` otherwise.
     */
    bool getVelocityChassisDisplacement(
        modm::Vector3f* const velocity,
        modm::Vector3f* const displacement) const final;

private:
    tap::control::chassis::ChassisSubsystemInterface* chassis;
    modm::Location2D<float> absoluteDisplacement;
    modm::Vector2f velocity;
    // Previous time at which motors are all online. 0 iff motors were offline last update.
    uint32_t prevTime = 0;
    // Stores whether or not stored displacement is valid. `True` after first good update.
    bool dataValid;
};

}  // namespace aruwsrc::algorithms::odometry

#endif  // OTTO_CHASSIS_VELOCITY_SLIPPAGE_DISPLACEMENT_2D_OBSERVER_HPP_
