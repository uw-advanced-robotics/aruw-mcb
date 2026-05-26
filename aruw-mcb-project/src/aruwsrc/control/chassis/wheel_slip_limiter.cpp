/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "wheel_slip_limiter.hpp"
#define DT 0.0002
namespace aruwsrc::control::chassis
{
WheelSlipLimiter::WheelSlipLimiter(
    tap::Drivers* drivers,
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
    const WheelSlipSVM& svm)
    : Subsystem(drivers),
      chassisYawObserver(chassisYawObserver),
      svm(svm)
{
}

// current implementation of this will assume all variables apart from translational acceleration
// are valid, then calculate max translational acceleration from those values. Currently NO error
// handling whatsoever so as long as this comment exists, this has potential to blow up the robot or
// display some other form of undefined behavior. :)
ChassisDriveOutput WheelSlipLimiter::getConstrainedOutput(ChassisDriveOutput request)
{
    float prevChassisTranslationVel =
        std::hypot(prevChassisXDesiredWheelspeed, prevChassisYDesiredWheelspeed);
    float reqTranslationVel =
        std::hypot(request.chassisXDesiredWheelspeed, request.chassisYDesiredWheelspeed);
    float reqTranslationAccel = (reqTranslationVel - prevChassisTranslationVel) / 0.002;
    float reqRotationAccel =
        (request.chassisRotationDesiredWheelspeed - prevChassisRotationDesiredWheelspeed) / DT;

    if (!svm.predict(std::array<double, 4>{
            reqTranslationVel,
            request.chassisRotationDesiredWheelspeed,
            reqTranslationAccel,
            reqRotationAccel}))  // predict returns true when it slips
    {
        prevChassisXDesiredWheelspeed = request.chassisXDesiredWheelspeed;
        prevChassisYDesiredWheelspeed = request.chassisYDesiredWheelspeed;
        prevChassisRotationDesiredWheelspeed = request.chassisRotationDesiredWheelspeed;
        return request;
    }
    if (!chassisYawObserver.getChassisWorldYaw(&chassisYaw))
    {
        return request;
    }
    float constrainedAccel = svm.findMaxSafeParam(
        2,
        std::array<double, 4>{
            prevChassisTranslationVel,
            request.chassisRotationDesiredWheelspeed,
            0,
            reqRotationAccel});
    if (constrainedAccel < 0.0f || std::isinf(constrainedAccel))
    {
        prevChassisXDesiredWheelspeed = request.chassisXDesiredWheelspeed;
        prevChassisYDesiredWheelspeed = request.chassisYDesiredWheelspeed;
        prevChassisRotationDesiredWheelspeed = request.chassisRotationDesiredWheelspeed;
        return request;
    }
    ChassisDriveOutput output = {
        (prevChassisTranslationVel + constrainedAccel * DT) * std::sin(chassisYaw),
        (prevChassisTranslationVel + constrainedAccel * DT) * std::cos(chassisYaw),
        request.chassisRotationDesiredWheelspeed};
    prevChassisXDesiredWheelspeed = output.chassisXDesiredWheelspeed;
    prevChassisYDesiredWheelspeed = output.chassisYDesiredWheelspeed;
    prevChassisRotationDesiredWheelspeed = output.chassisRotationDesiredWheelspeed;
    return output;
}
}  // namespace aruwsrc::control::chassis
