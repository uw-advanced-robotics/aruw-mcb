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
#ifndef WHEEL_HPP_
#define WHEEL_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

#include "modm/container/pair.hpp"
#include "modm/math/filter/pid.hpp"
using Motor = tap::motor::DjiMotor;
using SmoothPid = tap::algorithms::SmoothPid;
using SmoothPidConfig = tap::algorithms::SmoothPidConfig;
namespace aruwsrc
{
namespace chassis
{
// create a struct with wheel pose/radius/orientation
struct WheelConfig
{
    float wheelPositionChassisRelativeX;
    float wheelPositionChassisRelativeY;
    float wheelOrientationChassisRelative;
    SmoothPidConfig& velocityPidConfig;
    bool isPowered = true;
};

class Wheel
{
public:
    /* Creates a wheel object using given motorId, x-direction distance from chassis center,
        y-direction distance from chassis center, wheel orientation, if wheel is powered
    */
    Wheel(Motor& driveMotor, WheelConfig& config);

    /**
     * Calculates desired x and y velocity of the wheel based on passed in x, y, and r
     * components of the chassis velocity
     *
     * @param[in] vx The desired velocity of the chassis to move in the x direction in m/s
     * @param[in] vy The desired velocity of the chassis to move in the y direction in m/s
     * @param[in] vr The desired rotational velocity of the chassis in rpm
     *
     * @return a float Pair with the first value containing the desired velocity of the wheel
     *         in the x direction and the second value containing the desired velocity
     *         of the wheel in the y direction. Units: m/s. Might change type later???
     */
    virtual modm::Pair<float, float> calculateDesiredWheelVelocity(
        float vx,
        float vy,
        float vr) = 0;

    /**
     * Updates the desired wheel RPM based on passed in x and y components of desired
     * wheel velocity
     * @param[in] vx The desired velocity of the wheel to move in the x direction in m/s
     * @param[in] vy The desired velocity of the wheel to move in the y direction in m/s
     */
    virtual void executeWheelVelocity(float vx, float vy) = 0;

private:
    // Motor that drives the wheel
    Motor& motor;
    // PID used to control the driving motor
    SmoothPid velocityPid;
    // Whether or not the wheel is driven
    WheelConfig config;

};  // class Wheel
}  // namespace chassis
}  // namespace aruwsrc

#endif  // WHEEL_HPP_
