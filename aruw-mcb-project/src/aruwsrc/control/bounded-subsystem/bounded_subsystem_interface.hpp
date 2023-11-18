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

#ifndef BOUNDED_SUBSYSTEM_INTERFACE_HPP_
#define BOUNDED_SUBSYSTEM_INTERFACE_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::control
{
/**
 * Interface for a homeable and bounded subsystem, which is a subsytem where its one motor
 * is both homeable and constrained to a specific bounded axis.
 *
 * The lower bound is the furthest the motor is allowed to move in one (arbitrary) direction along its axis 
 * of movement and the upper bound is the furthest it can move in the opposite direction.
 */

class BoundedSubsystemInterface : public tap::control::Subsystem
{
public:
    BoundedSubsystemInterface(tap::Drivers* drivers) : Subsystem(drivers) {}
    
    /**
     * Finds and sets the home and bounds. Upon completion of this function,
     * homedAndBounded() will now return true.
    */
    virtual void calibrate() = 0;

    /**
     * Returns whether or not the home and bounds have been set.
     */
    virtual bool homedAndBounded() const = 0;

    /**
     * Returns the upper bound in motor encoder ticks.
    */
    virtual uint64_t getUpperBound() const = 0;

    /**
     * Returns the lower bound in motor encoder ticks.
    */
    virtual uint64_t getLowerBound() const = 0;

protected:

    /**
     * Stops the motor from moving. Only to be used during calibration.
     */
    virtual void haltHoming() = 0;

    /**
     * Sets the lower bound of this bounded subsystem to the given encoder position.
     */
    virtual void setLowerBound(uint64_t encoderPosition) = 0;

    /**
     * Sets the upper bound of this bounded subsystem to the given encoder position.
     */
    virtual void setUpperBound(uint64_t encoderPosition) = 0;

    /**
     * Sets the given motor encoder position to now be the "home" of the subsystem's motor.
    */
    virtual void setHome(uint64_t encoderPosition) = 0;
};  // class HomeableSubsystemInterface
}  // namespace aruwsrc::control

#endif  // HOMEABLE_SUBSYSYSTEM_INTERFACE_HPP
