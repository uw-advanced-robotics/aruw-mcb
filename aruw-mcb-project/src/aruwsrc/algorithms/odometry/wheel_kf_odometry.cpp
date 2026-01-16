/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "wheel_kf_odometry.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"

namespace aruwsrc::algorithms::odometry
{
FourWheelKFOdometry::FourWheelKFOdometry(
    const tap::motor::DjiMotor *chassisMotors[4],
    const ChassisWheelConfig *chassisWheelConfigs[4],
    tap::algorithms::odometry::ChassisWorldYawObserverInterface& chassisYawObserver,
    tap::communication::sensors::imu::ImuInterface& imu,
    const modm::Vector2f initPos)
    : kf(KF_A, KF_C, KF_Q, KF_R, KF_P0),
      chassisYawObserver(chassisYawObserver),
      imu(imu),
      initPos(initPos),
      chassisAccelerationToMeasurementCovarianceInterpolator(
          CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT,
          MODM_ARRAY_SIZE(CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT))
{
    // Copy motor and config pointers to member arrays
    for (int i = 0; i < 4; i++) {
        this->chassisMotors[i] = chassisMotors[i];
        this->chassisWheelConfigs[i] = chassisWheelConfigs[i];
    }
    reset();
}

void FourWheelKFOdometry::reset()
{
    float initialX[int(OdomState::NUM_STATES)] = {initPos.x, 0.0f, 0.0f, initPos.y, 0.0f, 0.0f};
    kf.init(initialX);
}

void FourWheelKFOdometry::update()
{
    if (!chassisYawObserver.getChassisWorldYaw(&chassisYaw))
    {
        chassisYaw = 0;
        return;
    }

    // Get individual wheel velocities and convert to linear velocities
    // for (int i = 0; i < 4; i++)
    // {
    //     motorVel = chassisMotors[i]->getEncoder()->getVelocity(); // rad/s
    //     wheelLinearVel = motorVel * chassisWheelConfigs[i]->wheelRadius; // m/s
        
    //     // Calculate wheel velocity components in chassis frame based on wheel orientation
    //     float wheelAngle = chassisWheelConfigs[i]->wheelOrientationToForwardRadians;

    //     float wheelVectorX = wheelLinearVel * cos(wheelAngle);
    //     float wheelVectorY = wheelLinearVel * sin(wheelAngle);

    //     tap::algorithms::rotateVector(&wheelVectorX, &wheelVectorY, serial::VisionCoprocessor::MCB_ROTATION_OFFSET + chassisYaw);

    //     y[int(OdomInput::VEL_X_1) + i * 2] = wheelVectorX;
    //     y[int(OdomInput::VEL_Y_1) + i * 2] = wheelVectorY;
    // }

    leftFrontMotorVel = chassisMotors[0]->getEncoder()->getVelocity();
    leftFrontWheelLinearVel = leftFrontMotorVel * chassisWheelConfigs[0]->wheelRadius;
    float leftFrontWheelAngle = chassisWheelConfigs[0]->wheelOrientationToForwardRadians;
    leftFrontMotorLinearVelX = leftFrontWheelLinearVel * cos(leftFrontWheelAngle);
    leftFrontMotorLinearVelY = leftFrontWheelLinearVel * sin(leftFrontWheelAngle);
    y[int(OdomInput::VEL_X_1)] = leftFrontMotorLinearVelX;
    y[int(OdomInput::VEL_Y_1)] = leftFrontMotorLinearVelY;

    leftBackMotorVel = chassisMotors[1]->getEncoder()->getVelocity();
    leftBackWheelLinearVel = leftBackMotorVel * chassisWheelConfigs[1]->wheelRadius;
    float leftBackWheelAngle = chassisWheelConfigs[1]->wheelOrientationToForwardRadians;
    leftBackMotorLinearVelX = leftBackWheelLinearVel * cos(leftBackWheelAngle);
    leftBackMotorLinearVelY = leftBackWheelLinearVel * sin(leftBackWheelAngle);
    y[int(OdomInput::VEL_X_2)] = leftBackMotorLinearVelX;
    y[int(OdomInput::VEL_Y_2)] = leftBackMotorLinearVelY;

    rightFrontMotorVel = chassisMotors[2]->getEncoder()->getVelocity();
    rightFrontWheelLinearVel = rightFrontMotorVel * chassisWheelConfigs[2]->wheelRadius;
    float rightFrontWheelAngle = chassisWheelConfigs[2]->wheelOrientationToForwardRadians;
    rightFrontMotorLinearVelX = rightFrontWheelLinearVel * cos(rightFrontWheelAngle);
    rightFrontMotorLinearVelY = rightFrontWheelLinearVel * sin(rightFrontWheelAngle);
    y[int(OdomInput::VEL_X_3)] = rightFrontMotorLinearVelX;
    y[int(OdomInput::VEL_Y_3)] = rightFrontMotorLinearVelY;

    rightBackMotorVel = chassisMotors[3]->getEncoder()->getVelocity();
    rightBackWheelLinearVel = rightBackMotorVel * chassisWheelConfigs[3]->wheelRadius;
    float rightBackWheelAngle = chassisWheelConfigs[3]->wheelOrientationToForwardRadians;
    rightBackMotorLinearVelX = rightBackWheelLinearVel * cos(rightBackWheelAngle);
    rightBackMotorLinearVelY = rightBackWheelLinearVel * sin(rightBackWheelAngle);
    y[int(OdomInput::VEL_X_4)] = rightBackMotorLinearVelX;
    y[int(OdomInput::VEL_Y_4)] = rightBackMotorLinearVelY;

    // Get IMU acceleration data in chassis frame
    y[int(OdomInput::ACC_X)] = imu.getAx();
    y[int(OdomInput::ACC_Y)] = imu.getAy();

    // Rotate acceleration from MCB frame to the world frame
    tap::algorithms::rotateVector(
        &y[int(OdomInput::ACC_X)],
        &y[int(OdomInput::ACC_Y)],
        aruwsrc::communication::serial::VisionCoprocessor::MCB_ROTATION_OFFSET + chassisYaw);

    // Calculate average chassis velocity for measurement covariance update
    modm::Matrix<float, 3, 1> chassisVelocity;
    chassisVelocity[0][0] = (y[int(OdomInput::VEL_X_1)] + y[int(OdomInput::VEL_X_2)] + 
                            y[int(OdomInput::VEL_X_3)] + y[int(OdomInput::VEL_X_4)]) / 4.0f;
    chassisVelocity[1][0] = (y[int(OdomInput::VEL_Y_1)] + y[int(OdomInput::VEL_Y_2)] + 
                            y[int(OdomInput::VEL_Y_3)] + y[int(OdomInput::VEL_Y_4)]) / 4.0f;
    chassisVelocity[2][0] = 0; // We don't use rotational velocity here

    // Update measurement covariance based on acceleration
    updateMeasurementCovariance(chassisVelocity);

    // Perform the Kalman filter update - the C matrix handles the transformation
    kf.performUpdate(y);

    // Update the location and velocity accessor objects with values from the state vector
    updateChassisStateFromKF(chassisYaw);
}

void FourWheelKFOdometry::updateChassisStateFromKF(float chassisYaw)
{
    const auto& x = kf.getStateVectorAsMatrix();

    // update odometry velocity and orientation
    velocity.x = x[int(OdomState::VEL_X)];
    velocity.y = x[int(OdomState::VEL_Y)];

    location.setOrientation(chassisYaw);
    location.setPosition(x[int(OdomState::POS_X)], x[int(OdomState::POS_Y)]);
}

void FourWheelKFOdometry::updateMeasurementCovariance(
    const modm::Matrix<float, 3, 1>& chassisVelocity)
{
    const uint32_t curTime = tap::arch::clock::getTimeMicroseconds();
    const uint32_t dt = curTime - prevTime;
    prevTime = curTime;

    // return to avoid weird acceleration spike on startup
    if (prevTime == 0)
    {
        return;
    }

    // compute acceleration
    chassisMeasuredDeltaVelocity.x = tap::algorithms::lowPassFilter(
        chassisMeasuredDeltaVelocity.x,
        chassisVelocity[0][0] - prevChassisVelocity[0][0],
        CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA);

    chassisMeasuredDeltaVelocity.y = tap::algorithms::lowPassFilter(
        chassisMeasuredDeltaVelocity.y,
        chassisVelocity[1][0] - prevChassisVelocity[1][0],
        CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA);

    prevChassisVelocity = chassisVelocity;

    // dt is in microseconds, acceleration is dv / dt, so to get an acceleration with units m/s^2,
    // convert dt in microseconds to seconds
    const float accelMagnitude =
        chassisMeasuredDeltaVelocity.getLength() * 1E6 / static_cast<float>(dt);

    const float velocityCovariance =
        chassisAccelerationToMeasurementCovarianceInterpolator.interpolate(accelMagnitude);

    // Set measurement covariance for all wheel velocity measurements
    // Higher acceleration means higher likelihood of wheel slippage
    for (int i = 0; i < 8; i++)  // 4 wheels * 2 components each
    {
        kf.getMeasurementCovariance()[i * static_cast<int>(OdomInput::NUM_INPUTS) + i] = 
            velocityCovariance;
    }
}

void FourWheelKFOdometry::overrideOdometryPosition(const float positionX, const float positionY)
{
    auto currKFState = kf.getStateVectorAsMatrix();

    float newState[int(OdomState::NUM_STATES)] = {
        positionX,
        currKFState[int(OdomState::VEL_X)],
        currKFState[int(OdomState::ACC_X)],
        positionY,
        currKFState[int(OdomState::VEL_Y)],
        currKFState[int(OdomState::ACC_Y)]};

    kf.init(newState);
}

}  // namespace aruwsrc::algorithms::odometry