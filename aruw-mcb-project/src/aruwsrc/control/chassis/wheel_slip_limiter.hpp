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

#ifndef WHEEL_SLIP_LIMITER_HPP_
#define WHEEL_SLIP_LIMITER_HPP_

#include "tap/algorithms/odometry/chassis_world_yaw_observer_interface.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/wheel_slip_svm.hpp"


namespace aruwsrc::control::chassis
{
struct ChassisDriveOutput
{
    float chassisXDesiredWheelspeed = 0.0;
    float chassisYDesiredWheelspeed = 0.0;
    float chassisRotationDesiredWheelspeed = 0.0;
};
class WheelSlipLimiter : public tap::control::Subsystem
{
public:
    WheelSlipLimiter(
        tap::Drivers* drivers,
        tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
        const WheelSlipSVM& svm);
    ChassisDriveOutput getConstrainedOutput(ChassisDriveOutput request);

private:
    float prevChassisXDesiredWheelspeed = 0.0;
    float prevChassisYDesiredWheelspeed = 0.0;
    float prevChassisRotationDesiredWheelspeed = 0.0;
    float chassisYaw = 0;
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver;
    const WheelSlipSVM& svm;
};
}  // namespace aruwsrc::control::chassis

#endif  // WHEEL_SLIP_LIMITER_HPP_