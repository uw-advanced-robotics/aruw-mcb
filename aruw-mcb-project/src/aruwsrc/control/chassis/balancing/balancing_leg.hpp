/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BALANCING_LEG_HPP_
#define BALANCING_LEG_HPP_

#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/motion/five_bar_linkage.hpp"
#include "aruwsrc/motor/tmotor_ak80-9.hpp"

namespace aruwsrc
{
namespace chassis
{
/**
 *
 */
class BalancingLeg
{
public:
    BalancingLeg(
        tap::Drivers* drivers,
        aruwsrc::control::motion::FiveBarLinkage* fivebar,
        tap::motor::MotorInterface* wheelMotor,
        const float wheelRadius,
        const tap::algorithms::SmoothPidConfig driveWheelPidConfig);

    /**
     *
     */
    inline void setDesiredHeight(float height) { zDesired = height; };

    /**
     *
     */
    inline void setDesiredTranslationSpeed(float speed) { vDesired = speed; }

    /**
     *
     */
    inline float getCurrentHeight() { return zCurrent; };

    /**
     *
     */
    inline float getCurrentTranslationSpeed() { return vCurrent; };

    /**
     *
     */
    void update();

private:
    const float WHEEL_RADIUS;

    aruwsrc::control::motion::FiveBarLinkage* fivebar;
    tap::motor::MotorInterface* driveWheel;

    tap::algorithms::SmoothPid driveWheelPid;

    float zDesired,          // m
        vDesired,            // m/s
        zCurrent,            // m
        vCurrent,            // m/s
        motorLinkAnglePrev,  // rad
        tl_Current;          // rad
};
}  // namespace chassis
}  // namespace aruwsrc

#endif  // BALANCING_LEG_HPP_
