/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TWO_SIDED_BOUNDED_SUBSYSTEM_INTERFACE_HPP_
#define TWO_SIDED_BOUNDED_SUBSYSTEM_INTERFACE_HPP_

#include "aruwsrc/control/bounded-subsystem/bounded_subsystem_interface.hpp"
#include "aruwsrc/control/bounded-subsystem/trigger/trigger_interface.hpp"

namespace aruwsrc::control
{
/**
 * A Bounded Subsystem whose home and bounds are set upon activating two triggers
 * on either end of the axis, one after the other.
*/
class TwoSidedBoundedSubsystemInterface : public BoundedSubsystemInterface
{
public:
    TwoSidedBoundedSubsystemInterface(tap::Drivers* drivers, 
                                      TriggerInterface& lowerTrigger, 
                                      TriggerInterface& upperTrigger) 
    : BoundedSubsystemInterface(drivers),
      lowerTrigger(lowerTrigger),
      upperTrigger(upperTrigger) {}

    void calibrate() override;
private:
    /**
     * Moves the motor along its axis towards the lower bound.
    */
    virtual void moveTowardLowerBound() = 0;
    
    /**
     * Moves the motor along its axis towards the upper bound.
    */
    virtual void moveTowardUpperBound() = 0;

    TriggerInterface& lowerTrigger;
    TriggerInterface& upperTrigger;
    uint64_t length;
};  // class OneSidedBoundedSubsystemInterface
}  // namespace aruwsrc::control

#endif  // TWO_SIDED_BOUNDED_SUBSYSTEM_INTERFACE_HPP_