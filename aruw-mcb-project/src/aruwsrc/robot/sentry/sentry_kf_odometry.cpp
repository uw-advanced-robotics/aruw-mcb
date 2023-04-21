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

namespace aruwsrc::algorithms::odometry
{
SentryKFOdometry::SentryKFOdometry(
    tap::Drivers& drivers,
    const aruwsrc::chassis::HolonomicChassisSubsystem& chassis,
    const aruwsrc::control::turret::SentryTurretMajorSubsystem& turretMajor,
    const aruwsrc::control::turret::SentryTurretMinorSubsystem& turretMinorLeft,
    const aruwsrc::control::turret::SentryTurretMinorSubsystem& turretMinorRight)
    : drivers(drivers),
      chassis(chassis),
      turretMajor(turretMajor),
      turretMinorLeft(turretMinorLeft),
      turretMinorRight(turretMinorRight),
      kf(KF_A, KF_C, KF_Q, KF_R, KF_P0)
{
}

float SentryKFOdometry::getChassisYaw() { return drivers.mpu6500.getYaw(); }

float SentryKFOdometry::getMajorYaw() { return turretMajor.getWorldYaw(); }

float SentryKFOdometry::getLeftMinorYaw() { return turretMinorLeft.getWorldYaw(); }

float SentryKFOdometry::getLeftMinorPitch() { return turretMinorLeft.getWorldPitch(); }

float SentryKFOdometry::getRightMinorYaw() { return turretMinorRight.getWorldYaw(); }

float SentryKFOdometry::getRightMinorPitch() { return turretMinorRight.getWorldPitch(); }

void SentryKFOdometry::update()
{
    // Get chassis positional values
    modm::Matrix<float, 3, 1> chassisPos = chassis.getActualVelocityChassisRelative();
    // use the chassis imu to get yaw
    chassis.getVelocityWorldRelative(chassisPos, drivers.mpu6500.getYaw());

    float accData[3] = {drivers.mpu6500.getAx(), drivers.mpu6500.getAy(), drivers.mpu6500.getAz()};
    modm::Matrix<float, 3, 1> chassisAcc(accData);

    float newOdomInput[int(SentryKFOdometry::OdomInput::NUM_INPUTS)] = {
        chassisPos[0][0],
        chassisAcc[0][0],
        chassisPos[1][0],
        chassisAcc[1][0],
    };

    kf.performUpdate(newOdomInput);
}

}  // namespace aruwsrc::algorithms::odometry
