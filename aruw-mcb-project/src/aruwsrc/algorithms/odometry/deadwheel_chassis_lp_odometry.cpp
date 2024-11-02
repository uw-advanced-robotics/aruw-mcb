/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "deadwheel_chassis_lp_odometry.hpp"

namespace aruwsrc::algorithms::odometry
{
DeadwheelChassisLPOdometry::DeadwheelChassisLPOdometry(
    const aruwsrc::algorithms::odometry::TwoDeadwheelOdometryObserver& deadwheelOdometry,
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
    tap::communication::sensors::imu::ImuInterface& imu,
    const modm::Vector2f initPos,
    const float parallelCenterToWheelDistance,
    const float parallelWheelChassisRelativeAngleRadians,
    const float perpendicularWheelChassisRelativeAngleRadians)
    : deadwheelOdometry(deadwheelOdometry),
      chassisYawObserver(chassisYawObserver),
      imu(imu),
      initPos(initPos),
      parallelCenterToWheelDistance(parallelCenterToWheelDistance),
      parallelWheelChassisRelativeAngleRadians(parallelWheelChassisRelativeAngleRadians),
      perpendicularWheelChassisRelativeAngleRadians(perpendicularWheelChassisRelativeAngleRadians)
{
    reset();
}

void DeadwheelChassisLPOdometry::reset()
{
    float initialX[int(OdomState::NUM_STATES)] = {initPos.x, 0.0f, 0.0f, initPos.y, 0.0f, 0.0f};
}

void DeadwheelChassisLPOdometry::update()
{
    if (!chassisYawObserver.getChassisWorldYaw(&chassisYaw))
    {
        chassisYaw = 0;
        return;
    }

    // Assuming getPerpendicularWheelVelocity() and getParallelWheelVelocity() return the velocities
    // of the two omni wheels
    float rawV1 = deadwheelOdometry.getPerpendicularRPM();
    float rawV2 = deadwheelOdometry.getParallelMotorRPM();
    float V1 = deadwheelOdometry.rpmToMetersPerSecond(rawV1);
    float V2 = deadwheelOdometry.rpmToMetersPerSecond(rawV2);

    // Calculate velocities in the robot's frame of reference
    // Correct for rotation of the robot
    V2 -= modm::toRadian(imu.getGz()) * parallelCenterToWheelDistance;
    // Rotate the velocities based on the wheel rotations
    float Vx = (((V1 - V2)) * parallelWheelChassisRelativeAngleRadians);
    float Vy = (((V1 + V2)) * perpendicularWheelChassisRelativeAngleRadians);

    tap::algorithms::rotateVector(&Vx, &Vy, chassisYaw);

    // Get acceleration from IMU
    float ax = imu.getAx();
    float ay = imu.getAy();

    // Rotate acceleration to the world frame
    static float accelXWorld, accelYWorld;
    tap::algorithms::rotateVector(&ax, &ay, chassisYaw);
    accelXWorld = ax;
    accelYWorld = ay;

    // Create the measurement vector
    float y[int(OdomInput::NUM_INPUTS)] = {Vx, accelXWorld, Vy, accelYWorld};

    // Perform the low pass filter update
    updateChassisStateWithLowPassFilter(Vx, Vy);
}

void DeadwheelChassisLPOdometry::overrideOdometryPosition(modm::Vector2f& newPos)
{
    filteredLocation.setPosition(newPos.x, newPos.y);
}

void DeadwheelChassisLPOdometry::overrideOdometryOrientation(float deltaYaw)
{
    chassisYaw += deltaYaw;
    tap::algorithms::rotateVector(&filteredVelocity.x, &filteredVelocity.y, chassisYaw);
}

void DeadwheelChassisLPOdometry::updateChassisStateWithLowPassFilter(float Vx, float Vy)
{
    // Apply low pass filter to velocities
    static float filteredVx = 0.0f;
    static float filteredVy = 0.0f;

    filteredVx = tap::algorithms::lowPassFilter(filteredVx, Vx, CHASSIS_VELOCITY_LOW_PASS_ALPHA);
    filteredVy = tap::algorithms::lowPassFilter(filteredVy, Vy, CHASSIS_VELOCITY_LOW_PASS_ALPHA);

    // Update the filtered velocity and position
    filteredVelocity.x = filteredVx;
    filteredVelocity.y = filteredVy;

    // Assuming a simple integration for position update
    static float prevTime = tap::arch::clock::getTimeMicroseconds();
    float curTime = tap::arch::clock::getTimeMicroseconds();
    float dt = (curTime - prevTime) * 1E-6; // Convert microseconds to seconds
    prevTime = curTime;

    filteredLocation.setPosition(
        filteredLocation.getX() + filteredVx * dt,
        filteredLocation.getY() + filteredVy * dt);
    filteredLocation.setOrientation(chassisYaw);
}

}