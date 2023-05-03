/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef HOMEABLE_SUBSYSTEM_INTERFACE_HPP_
#define HOMEABLE_SUBSYSTEM_INTERFACE_HPP_

#include "tap/control/subsystem.hpp"

namespace aruwsrc::control
{

struct HomingConfig {
    int16_t minRPM;
    int16_t maxRPM;
    int16_t minTorque;
    int16_t maxTorque;
};

/**
 * Interface for finding the bounds of a homeable subsystem.
 *
 * The lower bound is the furthest the motor can physically move in one direction along its axis of
 * movement and the upper bound is the furthest it can move in the opposite direction.
 */
class HomeableSubsystemInterface : public tap::control::Subsystem
{
public:
    HomeableSubsystemInterface(tap::Drivers* drivers) : Subsystem(drivers) {}

    /**
     * Moves the motor on the homeeable axis towards its upper mechanical limit.
    */
    virtual void moveTowardUpperBound() = 0;

    /**
     * Moves the motor on the homeable axis towards its lower mechanical limit.
    */
    virtual void moveTowardLowerBound() = 0;
    
    /**
     * Stops the motor on the homeable axis.
    */
    virtual void stop() = 0;

    /**
     * Detects whether the subsystem's motor is stalled, indicating that it has reached a hard stop.
     */
    virtual bool isStalled() const = 0;

    /**
     * Sets the lower bound of this homeable subsystem's home for the motor at its current position.
     */
    virtual void setLowerBound() = 0;

    /**
     * Sets the upper bound of this homeable subsystem's home for the motor at its current position.
     */
    virtual void setUpperBound() = 0;
};
}  // namespace aruwsrc::control

#endif
