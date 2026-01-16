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

#ifndef HOMEABLE_SUBSYSTEM_INTERFACE_HPP_
#define HOMEABLE_SUBSYSTEM_INTERFACE_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"

#include "trigger/trigger_interface.hpp"

namespace aruwsrc::control::joint::homing
{
/**
 * Interface for a homeable and bounded subsystem, which is a subsytem where its motor
 * is both homeable and constrained to a specific bounded axis.
 *
 * The lower bound is defined as the furthest the motor is allowed to move in one (arbitrary)
 * direction along its axis of movement and the upper bound is the furthest it can move in the
 * opposite direction.
 */

class HomeableSubsystemInterface : public virtual tap::control::Subsystem
{
public:
    // upper trigger is a pointer, so that it can be null if your subsystem is only bounded on one
    // side
    HomeableSubsystemInterface(
        tap::Drivers* drivers,
        trigger::TriggerInterface& lowerTrigger,
        trigger::TriggerInterface* upperTrigger = nullptr)
        : Subsystem(drivers),
          lowerTrigger(lowerTrigger),
          upperTrigger(upperTrigger),
          trigger(lowerTrigger)
    {
    }

    /**
     * Starts the calibration. Sets CalibrationState to CALIBRATING_LOWER_BOUND.
     * The actual calibrating logic occurs in refresh() of the base class.
     */
    void startCalibrate() { calibrationState = CalibrationState::CALIBRATING_LOWER_BOUND; }

    /**
     * Returns whether or not the home and bounds have been set.
     */
    virtual bool homedAndBounded() const = 0;

    /**
     * Returns the upper bound.
     */
    virtual float getUpperBound() const = 0;

    /**
     * Returns the lower bound.
     */
    virtual float getLowerBound() const = 0;

protected:
    /**
     * Specifies the current calibration state that command is in. Use in refresh() of child class.
     */
    enum class CalibrationState
    {
        AWAITING_CALIBRATE,  // not yet calibrating: call startCalibrate() to start.
        CALIBRATING_LOWER_BOUND,
        CALIBRATING_UPPER_BOUND,  // one-sided does not use this
        CALIBRATION_COMPLETE      // calibration done
    };

    CalibrationState calibrationState;

    /**
     * Moves the motor along its axis towards the lower bound.
     */
    virtual void moveTowardLowerBound() = 0;

    /**
     * Moves the motor along its axis towards the upper bound.
     * Defaulted to empty so that one sided systems don't need to override.
     */
    void moveTowardUpperBound() {}

    /**
     * Stops the motor from moving. Only to be used during calibration.
     */
    virtual void stopDuringHoming() = 0;
    /**
     * Sets the given position to be the "home" of the subsystem's motor.
     */
    virtual void setHome(float encoderPosition) = 0;

    trigger::TriggerInterface& lowerTrigger;
    trigger::TriggerInterface* upperTrigger;
    trigger::TriggerInterface& trigger;  // same as lower trigger, but easier for one sided systems
};                                       // class HomeableSubsystemInterface
}  // namespace aruwsrc::control::joint::homing

#endif  // HOMEABLE_SUBSYSYSTEM_INTERFACE_HPP
