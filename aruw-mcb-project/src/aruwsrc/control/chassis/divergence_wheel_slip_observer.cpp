/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "divergence_wheel_slip_observer.hpp"
#include "tap/algorithms/odometry/odometry_2d_tracker.hpp"
#include "tap/algorithms/math_user_utils.hpp"

namespace aruwsrc::control::chassis
{
    DivergenceWheelSlipObserver::DivergenceWheelSlipObserver(
        tap::Drivers* drivers,
        const tap::control::chassis::ChassisSubsystemInterface& chassisSubsystem,
        tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
        tap::communication::sensors::imu::ImuInterface& imu
    ) : Subsystem(drivers),
        chassisSubsystem(chassisSubsystem),
        chassisYawObserver(chassisYawObserver),
        imu(imu)
        {

        }
    
    void DivergenceWheelSlipObserver::update()
    {
        if (!chassisYawObserver.getChassisWorldYaw(&chassisYaw))
    {
        return;
    }

    const uint32_t currentTime = tap::arch::clock::getTimeMicroseconds();
    const float dt = (currentTime - prevTime) / 1'000'000.0f;  // Convert to seconds
    prevTime = currentTime;

    // Get chassis velocities
    float chassis_x_accel, chassis_y_accel;

    modm::Matrix<float, 3, 1> chassisVelocity = chassisSubsystem.getActualVelocityChassisRelative();
    tap::algorithms::odometry::getVelocityWorldRelative(chassisVelocity, chassisYaw);

    chassis_x_accel = (prev_chassis_x_accel - chassisVelocity[0][0]) / dt;
    chassis_y_accel = (prev_chassis_y_accel - chassisVelocity[1][0]) / dt;

    float imu_x_accel = imu.getAx();
    float imu_y_accel = imu.getAy();

    slipping = tap::algorithms::compareFloatClose(chassis_x_accel, imu_x_accel, 0) 
            && tap::algorithms::compareFloatClose(chassis_y_accel, imu_y_accel, 0);

    }

}