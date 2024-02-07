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
    MecanumWheel(Motor& driveMotor, const WheelConfig& config, bool invertAngle);

    void executeWheelVelocity(float vx, float vy) override;

    void initialize() override;
    void refresh() override;
    void setZeroRPM() override;
    bool allMotorsOnline() const override;
    float getDriveVelocity() const override;
    float getDriveRPM() const override;
    int getNumMotors() const override;

private:
    float driveSetPoint;
    bool invertAngle;
    Motor& driveMotor;
    // PID used to control the driving motor
    SmoothPid velocityPid;
    const float WHEEL_RELATIVE_TO_ROLLER_ANGLE = M_PI_4;
    const float AXLE_TO_ROBOT_FRONT = M_PI_2;
    CMSISMat<2, 2> MAT1;
    CMSISMat<2, 2> MAT2;
    /// product of matrices 1 and 2 in equation on Swerve! Notion
    CMSISMat<2, 2> PRODUCT_MAT;
    CMSISMat<2, 1> wheelMat;
};  // class MecanumWheel
}  // namespace chassis
}  // namespace aruwsrc

#endif  // MECANUM_WHEEL_HPP_
