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

#include "deadwheel_chassis_cf_odometry.hpp"

namespace aruwsrc::algorithms::odometry
{

DeadwheelChassisCFOdometry::DeadwheelChassisCFOdometry(
    const DeadwheelOdometryObserver& deadwheelOdometry,
    YawObserver& chassisYawObserver,
    ImuInterface& imu,
    const modm::Vector2f initPos,
    const float parallelCenterToWheelDistance,
    const float parallelWheelChassisForwardRelativeAngleRadians,
    const float perpendicularWheelChassisForwardRelativeAngleRadians)
    : deadwheelOdometry(deadwheelOdometry),
      chassisYawObserver(chassisYawObserver),
      imu(imu),
      initPos(initPos),
      parallelCenterToWheelDistance(parallelCenterToWheelDistance),
      parallelWheelChassisForwardRelativeAngleRadians(
          parallelWheelChassisForwardRelativeAngleRadians),
      perpendicularWheelChassisForwardRelativeAngleRadians(
          perpendicularWheelChassisForwardRelativeAngleRadians)
{
    reset();
}

void DeadwheelChassisCFOdometry::reset()
{
    location = modm::Location2D<float>(initPos.x, initPos.y, 0.0f);
    velocity = modm::Vector2f(0.0f, 0.0f);
    chassisYaw = 0.0f;
    prevTime = tap::arch::clock::getTimeMicroseconds();
}

void DeadwheelChassisCFOdometry::update()
{
    if (!chassisYawObserver.getChassisWorldYaw(&chassisYaw))
    {
        return;
    }

    const uint32_t currentTime = tap::arch::clock::getTimeMicroseconds();
    const float dt = (currentTime - prevTime) / 1'000'000.0f;  // Convert to seconds
    prevTime = currentTime;

    // Get deadwheel velocities
    float deadwheel_x_vel, deadwheel_y_vel;
    computeDeadwheelVelocities(&deadwheel_x_vel, &deadwheel_y_vel);

    // Get IMU acceleration data
    float acc_x_vel, acc_y_vel;
    computeAccVelocities(&acc_x_vel, &acc_y_vel, dt);

    // Complementary filter time!
    float velocity_x = deadwheelTrust * deadwheel_x_vel + (1 - deadwheelTrust) * acc_x_vel;
    float velocity_y = deadwheelTrust * deadwheel_y_vel + (1 - deadwheelTrust) * acc_y_vel;

    velocity = modm::Vector2f(velocity_x, velocity_y);
    float position_x = location.getX() + velocity.x * dt;
    float position_y = location.getY() + velocity.y * dt;
    location.setPosition(position_x, position_y);
    location.setOrientation(chassisYaw);
}

void DeadwheelChassisCFOdometry::computeDeadwheelVelocities(
    float* deadwheel_x_vel,
    float* deadwheel_y_vel) const
{
    float perpendicularWheelVelocity = deadwheelOdometry.getPerpendicularVelocity();
    float parallelWheelVelocity = deadwheelOdometry.getParallelMotorVelocity();
    // Adjust parallel wheel velocity based on chassis yaw
    parallelWheelVelocity += imu.getGz() * parallelCenterToWheelDistance;

    float x_vel =
        parallelWheelVelocity * std::sin(parallelWheelChassisForwardRelativeAngleRadians) +
        perpendicularWheelVelocity * std::cos(perpendicularWheelChassisForwardRelativeAngleRadians);
    float y_vel =
        parallelWheelVelocity * std::cos(parallelWheelChassisForwardRelativeAngleRadians) -
        perpendicularWheelVelocity * std::sin(perpendicularWheelChassisForwardRelativeAngleRadians);

    tap::algorithms::rotateVector(&x_vel, &y_vel, chassisYaw);
    *deadwheel_x_vel = x_vel;
    *deadwheel_y_vel = y_vel;
}

void DeadwheelChassisCFOdometry::computeAccVelocities(
    float* acc_x_vel,
    float* acc_y_vel,
    const float dt) const
{
    // Get IMU acceleration data
    float ax = imu.getAx();
    float ay = imu.getAy();
    tap::algorithms::rotateVector(&ax, &ay, chassisYaw);

    // Compute acceleration velocities
    *acc_x_vel = velocity.x + ax * dt;
    *acc_y_vel = velocity.y + ay * dt;
}

}  // namespace aruwsrc::algorithms::odometry
