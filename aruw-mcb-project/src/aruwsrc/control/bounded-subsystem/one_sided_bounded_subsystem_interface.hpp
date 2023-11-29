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

#ifndef ONE_SIDED_BOUNDED_SUBSYSTEM_INTERFACE_HPP_
#define ONE_SIDED_BOUNDED_SUBSYSTEM_INTERFACE_HPP_

#include "aruwsrc/control/bounded-subsystem/bounded_subsystem_interface.hpp"
#include "aruwsrc/control/bounded-subsystem/trigger/trigger_interface.hpp"

namespace aruwsrc::control
{
/**
 * A bounded subsystem whose home and both of its bounds are set upon activating a single 
 * trigger on one side of its axis. Its other bound is derived using its length.
 */
class OneSidedBoundedSubsystemInterface : public BoundedSubsystemInterface
{
public:
    OneSidedBoundedSubsystemInterface(
        tap::Drivers* drivers,
        TriggerInterface& trigger,
        uint64_t length)
        : BoundedSubsystemInterface(drivers),
          trigger(trigger),
          length(length)
    {
    }

protected:
    /**
     * Moves the motor along its axis towards the lower bound.
     */
    virtual void moveTowardLowerBound() = 0;

    TriggerInterface& trigger;

    /**
     * The length of the subsystem. Used to calculate the motor's upper bound.
     */
    uint64_t length;
};  // class OneSidedBoundedSubsystemInterface
}  // namespace aruwsrc::control

#endif  // ONE_SIDED_BOUNDED_SUBSYSTEM_INTERFACE_HPP_
