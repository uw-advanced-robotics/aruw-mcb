/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef FRICTION_WHEEL_INTERFACE_HPP_
#define FRICTION_WHEEL_INTERFACE_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::control::launcher
{
class FrictionWheelInterface : public tap::control::Subsystem
{
    // holder interface to hide templates

public:
    FrictionWheelInterface(tap::Drivers* drivers) : tap::control::Subsystem(drivers) {}

    float getPredictedLaunchSpeed() const;

    virtual void setDesiredLaunchSpeed(float speed, bool directRpm = true) = 0;

    virtual void setIndividualVelocity(int index, float velocity) = 0;

    virtual void changeWheelVelocityState(int index, bool hasIndividualVelocity) = 0;

    virtual float getDesiredLaunchSpeed() const = 0;

    virtual float getDesiredFrictionWheelSpeed() const = 0;

    virtual float getCurrentCorrectionValue() const = 0;

    virtual float getCurrentAverageFrictionWheelSpeed() const = 0;

    virtual float getCurrentIndividualFrictionWheelSpeed(int index) const = 0;

    virtual const char* getName() const = 0;
};
}  // namespace aruwsrc::control::launcher
#endif