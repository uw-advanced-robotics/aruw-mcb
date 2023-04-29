/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/robot/sentry/sentry_kf_odometry.hpp"

#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_drivers.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"

namespace aruwsrc::algorithms::odometry
{
SentryKFOdometry::SentryKFOdometry(
    aruwsrc::serial::VisionCoprocessor& visionCoprocessor,
    tap::communication::sensors::imu::ImuInterface& chassisIMU,
    const aruwsrc::chassis::HolonomicChassisSubsystem& chassis
    // const aruwsrc::control::turret::SentryTurretMajorSubsystem& turretMajor,
    // const aruwsrc::control::turret::SentryTurretMinorSubsystem& turretMinorLeft,
    // const aruwsrc::control::turret::SentryTurretMinorSubsystem& turretMinorRight)
    ) : 
        visionCoprocessor(visionCoprocessor),
        chassisIMU(chassisIMU),
        chassis(chassis),
        // turretMajor(turretMajor),
        // turretMinorLeft(turretMinorLeft),
        // turretMinorRight(turretMinorRight),
        kf(KF_A, KF_C, KF_Q, KF_R, KF_P0)
{
    kf.init({0, 0, 0, 0, 0, 0});
}

void SentryKFOdometry::initializeReferences(
    const aruwsrc::control::turret::SentryTurretMajorSubsystem& turretMajor,
    const aruwsrc::control::turret::SentryTurretMinorSubsystem& turretMinorLeft,
    const aruwsrc::control::turret::SentryTurretMinorSubsystem& turretMinorRight) 
{
    this->turretMajor = &turretMajor;
    this->turretMinorLeft = &turretMinorLeft;
    this->turretMinorRight = &turretMinorRight;
}

float SentryKFOdometry::getMajorYaw() { 
    return turretMajor->getWorldYaw() + this->turretMajorYawError; 
}

float SentryKFOdometry::getLeftMinorYaw() { 
    return turretMinorLeft->getWorldYaw() + leftMinorYawError; 
}

float SentryKFOdometry::getLeftMinorPitch() { return turretMinorLeft->getWorldPitch(); }

float SentryKFOdometry::getRightMinorYaw() { 
    return turretMinorRight->getWorldYaw() + rightMinorYawError; 
}

float SentryKFOdometry::getRightMinorPitch() { return turretMinorRight->getWorldPitch(); }

modm::Location2D<float> SentryKFOdometry::getCurrentLocation2D() const {
    auto state = kf.getStateVectorAsMatrix();
    modm::Location2D<float> loc;
    loc.setPosition(state[int(OdomState::POS_X)], state[int(OdomState::POS_Y)]);
    return loc;
}

/**
 * @return The current x and y velocity (in m/s).
 */
modm::Vector2f SentryKFOdometry::getCurrentVelocity2D() const {
    auto state = kf.getStateVectorAsMatrix();
    modm::Vector2f vel(state[int(OdomState::VEL_X)], state[int(OdomState::VEL_Y)]);
    return vel;
}

/**
 * @return The current yaw orientation of the chassis in the world frame in radians.
 */
float SentryKFOdometry::getYaw() const {
    return modm::toRadian(chassisIMU.getYaw());
}

/**
 * @return The last time that odometry was computed (in microseconds).
 */
uint32_t SentryKFOdometry::getLastComputedOdometryTime() const { return lastComputedOdometryTime; }


void SentryKFOdometry::update()
{
    // check if there is new odometry data to reset ours with
    // aruwsrc::serial::VisionCoprocessor::LocalizationCartesianData lastLocData = visionCoprocessor.getLastLocalizationData();
    // // uint32_t currentMessageTimestamp =  visionCoprocessor.getLastLocalizationData().timestamp;
    // if (lastResetTimestamp != UNINITIALIZED_TIMESTAMP 
    //         && lastResetTimestamp < lastLocData.timestamp) {
    //             resetOdometry(lastLocData);
    // }

    // Get chassis positional values
    modm::Matrix<float, 3, 1> chassisPos = chassis.getActualVelocityChassisRelative();
    // use the chassis imu to get yaw
    chassis.getVelocityWorldRelative(chassisPos, chassisIMU.getYaw());

    float accData[3] = {chassisIMU.getAx(), chassisIMU.getAy(), chassisIMU.getAz()};
    modm::Matrix<float, 3, 1> chassisAcc(accData);

    float newOdomInput[int(SentryKFOdometry::OdomInput::NUM_INPUTS)] = {
        chassisPos[0][0],
        chassisAcc[0][0],
        chassisPos[1][0],
        chassisAcc[1][0],
    };

    kf.performUpdate(newOdomInput);

    lastComputedOdometryTime = tap::arch::clock::getTimeMicroseconds();
}

// void SentryKFOdometry::resetOdometry(aruwsrc::serial::VisionCoprocessor::LocalizationCartesianData newData) {
//     // naively reset kalman filter
//     // (need transformer in order to transform minors to chassis position - some constant offset) 
//     // TODO: rotate velocity and acceleration to new frame
//     kf.init({newData.x, 0.0f, 0.0f, newData.y, 0.0f, 0.0f});

//     if (newData.turretID == 0) {
//         // left turret minor
//         float currentLeftMinorYaw = this->getLeftMinorYaw();
//         this->leftMinorYawError = currentLeftMinorYaw - newData.yaw;
//     } else {
//         // right turret minor
//         float currentRightMinorYaw = this->getRightMinorYaw();
//         this->rightMinorYawError =  currentRightMinorYaw - newData.yaw;
//     }
// }

}  // namespace aruwsrc::algorithms::odometry
