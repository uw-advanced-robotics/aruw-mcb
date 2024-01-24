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
#ifndef MECANUM_WHEEL_HPP_
#define MECANUM_WHEEL_HPP_

#include "tap/architecture/clock.hpp"

#include "wheel.hpp"

namespace aruwsrc
{
namespace chassis
{
class MecanumWheel : public Wheel
{
public:
    /* Creates an mecanum wheel object using given motor, wheel config, and smoothpid config
     */
    MecanumWheel(Motor& driveMotor, WheelConfig& config);

    void executeWheelVelocity(float vx, float vy) override;

private:
    /// time tracker for smoothpid
    double prevTime = 0;
    const double WHEEL_RELATIVE_TO_ROLLER_ANGLE = M_PI_4;
    const double AXLE_TO_ROBOT_FRONT = M_PI_2;
    const CMSISMat<2, 2> MAT1 = CMSISMat<2, 2>({0.0f,
                                                sin(WHEEL_RELATIVE_TO_ROLLER_ANGLE),
                                                (float) config.diameter / 2,
                                                cos(WHEEL_RELATIVE_TO_ROLLER_ANGLE)})
                                    .inverse();
    const CMSISMat<2, 2> MAT2 = CMSISMat<2, 2>({cos(AXLE_TO_ROBOT_FRONT),
                                                -sin(AXLE_TO_ROBOT_FRONT),
                                                sin(AXLE_TO_ROBOT_FRONT),
                                                cos(AXLE_TO_ROBOT_FRONT)})
                                    .inverse();
    /// product of matrices 1 and 2 in equation on Swerve! Notion
    const CMSISMat<2, 2> PRODUCT_MAT = MAT1 * MAT2;
};  // class MecanumWheel
}  // namespace chassis
}  // namespace aruwsrc

#endif  // MECANUM_WHEEL_HPP_
