/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
 * Wrapper class for a mechanism that uses a five-bar linkage
 * with links grounded on a robot chassis and a wheel motor
 * attached to the end-effector to drive one side of a two-wheeled robot.
 *
 * See @class BalancingChassisSubsystem for application of this mechanism.
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

    void initialize();

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

    inline modm::Vector2f getDefaultPosition() { return fivebar->getDefaultPosition(); }

    /**
     *
     */
    void update();

private:
    void computeState(uint32_t dt);
    const float WHEEL_RADIUS;

    aruwsrc::control::motion::FiveBarLinkage* fivebar;
    tap::motor::MotorInterface* driveWheel;

    tap::algorithms::SmoothPid driveWheelPid;

    tap::algorithms::SmoothPidConfig xPidConfig{
        .kp = .001,
        .ki = 0,
        .kd = 0,
        .maxOutput = .03,
    };
    tap::algorithms::SmoothPid xPid = tap::algorithms::SmoothPid(xPidConfig);
 
    uint32_t prevTime = 0;
    float wheelPosPrev = 0;
    float tl_prev = 0;

    float zDesired,          // m
        vDesired,            // m/s
        vDesiredPrev,
        zCurrent,            // m
        vCurrent,            // m/s
        vCurrentPrev,
        motorLinkAnglePrev,  // rad
        tl,                  // rad
        tl_dot,
        tl_dotPrev;              // rad/s
    float realWheelSpeed;
    float aCurrentPrev;
    float aCurrent;
    float aDesired;
    float aDesiredPrev;
    float xoffset;
    float xoffsetPrev;
};
}  // namespace chassis
}  // namespace aruwsrc

#endif  // BALANCING_LEG_HPP_
