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

#include "chassis_cf_odometry.hpp"

#include "tap/architecture/clock.hpp"

#include "aruwsrc/communication/serial/rtt_telemetry.hpp"

namespace aruwsrc::algorithms::odometry
{
ChassisCFOdometry::ChassisCFOdometry(
    tap::Drivers* drivers,
    const tap::control::chassis::ChassisSubsystemInterface& chassisSubsystem,
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
    tap::communication::sensors::imu::ImuInterface& imu,
    const modm::Vector2f initPos,
    aruwsrc::communication::serial::RttTelemetry* rttTelemetry)
    : Subsystem(drivers),
      chassisSubsystem(chassisSubsystem),
      chassisYawObserver(chassisYawObserver),
      imu(imu),
      initPos(initPos),
      rttTelemetry(rttTelemetry)
{
    reset();
}

void ChassisCFOdometry::reset()
{
    location = modm::Location2D<float>(initPos.x, initPos.y, 0.0f);
    velocity = modm::Vector2f(0.0f, 0.0f);
    chassisYaw = 0.0f;
    prevTime = tap::arch::clock::getTimeMicroseconds();
}

void ChassisCFOdometry::update()
{
    if (!chassisYawObserver.getChassisWorldYaw(&chassisYaw))
    {
        return;
    }

    const uint32_t currentTime = tap::arch::clock::getTimeMicroseconds();
    const float dt = (currentTime - prevTime) / 1'000'000.0f;  // Convert to seconds
    prevTime = currentTime;

    // Get chassis velocities
    float chassis_x_vel, chassis_y_vel;

    modm::Matrix<float, 3, 1> chassisVelocity = chassisSubsystem.getActualVelocityChassisRelative();
    tap::control::chassis::ChassisSubsystemInterface::getVelocityWorldRelative(
        chassisVelocity,
        chassisYaw);

    chassis_x_vel = chassisVelocity[0][0];
    chassis_y_vel = chassisVelocity[1][0];

    // Get IMU acceleration data
    float acc_x_vel, acc_y_vel;
    computeAccVelocities(&acc_x_vel, &acc_y_vel, dt);

    // Complementary filter time!
    float velocity_x = chassisTrust * chassis_x_vel + (1.0 - chassisTrust) * acc_x_vel;
    float velocity_y = chassisTrust * chassis_y_vel + (1.0 - chassisTrust) * acc_y_vel;

    velocity = modm::Vector2f(velocity_x, velocity_y);
    float position_x = location.getX() + velocity.x * dt;
    float position_y = location.getY() + velocity.y * dt;
    location.setPosition(position_x, position_y);
    location.setOrientation(chassisYaw);

    // Log odometry data to RTT telemetry if available
    if (rttTelemetry != nullptr)
    {
        float posArr[2] = {position_x, position_y};
        rttTelemetry->logSignal<float, POS_LOG_ID, 2>(posArr);
    }
}

void ChassisCFOdometry::computeAccVelocities(float* acc_x_vel, float* acc_y_vel, const float dt)
{
    // Get IMU acceleration data
    float acc_x = imu.getAx();
    float acc_y = imu.getAy();

    // Rotate to world frame
    tap::algorithms::rotateVector(&acc_x, &acc_y, imu.getYaw());

    // Get current velocity
    float curr_x_vel = velocity.x;
    float curr_y_vel = velocity.y;

    // Update velocity
    *acc_x_vel = curr_x_vel + acc_x * dt;
    *acc_y_vel = curr_y_vel + acc_y * dt;
}

}  // namespace aruwsrc::algorithms::odometry
