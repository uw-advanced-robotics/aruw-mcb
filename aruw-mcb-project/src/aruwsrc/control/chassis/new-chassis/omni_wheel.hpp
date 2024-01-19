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
#ifndef OMNI_WHEEL_HPP_
#define OMNI_WHEEL_HPP_

#include "wheel.hpp"

namespace aruwsrc
{
namespace chassis
{
class OmniWheel : public Wheel
{
public:
    /* Creates an omni wheel object using given motorId, x-direction distance from chassis center,
        y-direction distance from chassis center, wheel orientation, if wheel is powered
    */
    OmniWheel(Motor& driveMotor, WheelConfig& config, SmoothPidConfig& wheelPIDConfig);

    const double WHEEL_RELATIVE_ROLLER_ANGLE = M_PI_2;
    const double AXLE_TO_ROBOT_FRONT = M_PI_2;

    void executeWheelVelocity(float vx, float vy) override;
    modm::Pair<float, float> calculateDesiredWheelVelocity(float vx, float vy, float vr) override;

private:
    // Motor that drives the wheel
    tap::motor::DjiMotor& motor;
    // PID used to control the driving motor
    tap::algorithms::SmoothPid velocityPid;
    // config for the wheel PID controller
    WheelConfig config;
    // product of matrices 1 and 2 in equation on Swerve! Notion
    tap::algorithms::CMSISMat<2, 2> productMat;
    // matrix containing distances from wheel to chassis center
    tap::algorithms::CMSISMat<2, 3> distanceMat;
    // time tracker for smoothpid
    double prevTime = 0;
};  // class OmniWheel
}  // namespace chassis
}  // namespace aruwsrc

#endif  // OMNI_WHEEL_HPP_
