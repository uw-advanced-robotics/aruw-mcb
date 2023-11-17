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

#include "aruwsrc/control/homeable-subsystem/bounded_subsystem_interface.hpp"

namespace aruwsrc::control
{
class OneSidedHomeableSubsystemInterface : public HomeableSubsystemInterface
{
public:
    OneSidedHomeableSubsystemInterface(tap::Drivers* drivers, uint64_t length) 
    : HomeableSubsystemInterface(drivers),
      length(length) {}


    /**
     * Sets the upper bound of this bounded subsystem to the given encoder position.
     */
    virtual void setUpperBound(uint64_t encoderPosition) {
        // upper bound = length + lower bound
    }
}
private:
    uint64_t length;
}

#endif