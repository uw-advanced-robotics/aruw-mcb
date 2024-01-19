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
#include "mecanum_wheel.hpp"

namespace aruwsrc
{
namespace chassis
{
MecanumWheel::MecanumWheel(Motor& driveMotor, WheelConfig& config, SmoothPidConfig& wheelPIDConfig)
    : Wheel(driveMotor, config),
      motor(driveMotor),
      config(config),
      velocityPid(SmoothPid(wheelPIDConfig))
{
    tap::algorithms::CMSISMat<2, 2> mat1 =
        tap::algorithms::CMSISMat<2, 2>({0.0, 1.0, config.diameter / 2, 0.0});
    mat1 = mat1.inverse();
    tap::algorithms::CMSISMat<2, 2> mat2 = tap::algorithms::CMSISMat<2, 2>({0.0, -1.0, 1.0, 0.0});
    mat2 = mat2.inverse();
    productMat = mat1 * mat2;
    distanceMat = tap::algorithms::CMSISMat<2, 3>(
        {1,
         0,
         -1 * config.wheelPositionChassisRelativeY,
         0,
         1,
         config.wheelPositionChassisRelativeX});
}

modm::Pair<float, float> MecanumWheel::calculateDesiredWheelVelocity(float vx, float vy, float vr)
{
    tap::algorithms::CMSISMat<3, 1> chassisVel = tap::algorithms::CMSISMat<3, 1>({vx, vy, vr});
    tap::algorithms::CMSISMat<2, 1> wheelVel = distanceMat * chassisVel;
    return {wheelVel.data[0], wheelVel.data[1]};
}

void MecanumWheel::executeWheelVelocity(float vx, float vy)
{
    tap::algorithms::CMSISMat<2, 1> desiredMat =
        productMat * tap::algorithms::CMSISMat<2, 1>({vx, vy});
    double currentTime = tap::arch::clock::getTimeMicroseconds();
    motor.setDesiredOutput(velocityPid.runControllerDerivateError(
        desiredMat.data[0] - motor.getShaftRPM(),
        currentTime - prevTime));
    prevTime = currentTime;
}

}  // namespace chassis
}  // namespace aruwsrc
