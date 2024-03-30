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
MecanumWheel::MecanumWheel(Motor& driveMotor, WheelConfig& config) : Wheel(driveMotor, config)
// velocityPid(SmoothPid(config.velocityPidConfig))
{
}

void MecanumWheel::executeWheelVelocity(float vx, float vy)
{
    CMSISMat<2, 1> desiredMat = PRODUCT_MAT * CMSISMat<2, 1>({vx, vy});
    double currentTime = tap::arch::clock::getTimeMicroseconds();
    double error = desiredMat.data[0] - motor.getShaftRPM();
    motor.setDesiredOutput(velocityPid.runControllerDerivateError(error, currentTime - prevTime));
    prevTime = currentTime;
}

void MecanumWheel::initialize()
{
    if (config.isPowered)
    {
        motor.initialize();
    }
}

}  // namespace chassis
}  // namespace aruwsrc
