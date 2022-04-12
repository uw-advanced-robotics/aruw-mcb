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

#ifndef OTTO_CHASSIS_DISPLACEMENT_OBSERVER
#define OTTO_CHASSIS_DISPLACEMENT_OBSERVER

#include "tap/algorithms/odometry/chassis_displacement_observer_interface.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"

#include "modm/math/geometry/location_2d.hpp"
#include "modm/math/geometry/vector.hpp"

namespace aruwsrc::algorithms::odometry
{
/**
 * Chassis displacement observer based directly on observed wheel angle displacements.
 * 
 * Relies on being updated frequently.
 */
class OttoChassisDisplacementObserver
    : public tap::algorithms::odometry::ChassisDisplacementObserverInterface
{
public:
    /**
     * @param[in] chassis the chassis object to get wheel angles from (for determining displacement)
     */
    OttoChassisDisplacementObserver(
        const tap::control::chassis::ChassisSubsystemInterface& chassis);

    /**
     * Update the observed displacement. Call frequently for best results.
     */
    void update();

    /** See ChassisDisplacementObserverInterface::getChassisDisplacement() for details */
    bool getChassisDisplacement(modm::Vector<float, 3>* displacement) const final;

private:
    const tap::control::chassis::ChassisSubsystemInterface& chassis;
};

}  // namespace aruwsrc::algorithms::odometry

#endif  // OTTO_CHASSIS_DISPLACEMENT_OBSERVER
