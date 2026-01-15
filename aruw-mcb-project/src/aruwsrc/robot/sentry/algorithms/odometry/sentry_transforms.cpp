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

#include "sentry_transforms.hpp"

using namespace tap::algorithms::odometry;
using namespace tap::algorithms::transforms;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::sentry::turret;

using tap::communication::sensors::imu::ImuInterface;

namespace aruwsrc::sentry::algorithms::odometry
{
SentryTransforms::SentryTransforms(
    const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry,
    const YawTurretSubsystem& turretMajor,
#ifdef TARGET_SENTINEL_2026
    const SentryTurretMinorSubsystem& turretWidow,
    const ImuInterface& turretWidowImu,
#else
    const SentryTurretMinorSubsystem& turretLeft,
    const ImuInterface& turretLeftImu,
    const SentryTurretMinorSubsystem& turretRight,
    const ImuInterface& turretRightImu,
#endif
    const SentryTransforms::SentryTransformConfig& config)
    : config(config),
      chassisOdometry(chassisOdometry),
      turretMajor(turretMajor),
#ifdef TARGET_SENTINEL_2026
      turretWidow(turretWidow),
      turretWidowImu(turretWidowImu),
#else
      turretLeft(turretLeft),
      turretLeftImu(turretLeftImu),
      turretRight(turretRight),
      turretRightImu(turretRightImu),
#endif
      worldToChassis(Transform::identity()),
      worldToTurretMajor(Transform::identity()),
#ifdef TARGET_SENTINEL_2026
      worldToTurretWidow(Transform::identity()),
      turretWidowYawSyncPid(config.imuSyncConfig),
      turretWidowYawCorrection(0),
#else
      worldToTurretLeft(Transform::identity()),
      turretLeftYawSyncPid(config.imuSyncConfig),
      turretLeftYawCorrection(0),
      worldToTurretRight(Transform::identity()),
      turretRightYawSyncPid(config.imuSyncConfig),
      turretRightYawCorrection(0),
#endif
      worldToVTM(Transform::identity()),
      chassisToArducam0(Transform::identity()),
      chassisToArducam1(Transform::identity()),
      chassisToArducam2(Transform::identity()),
      chassisToArducam3(Transform::identity()),
      chassisToTurretMajor(Transform::identity()),
#ifdef TARGET_SENTINEL_2026
      turretMajorToTurretWidow(0., config.turretMinorOffset, 0., 0., 0., 0.)
#else
      turretMajorToTurretLeft(0., config.turretMinorOffset, 0., 0., 0., 0.),
      turretMajorToTurretRight(0., -config.turretMinorOffset, 0., 0., 0., 0.)
#endif
{
}

void SentryTransforms::updateTransforms()
{
    // pose of chassis in world frame
    modm::Location2D chassisPose = chassisOdometry.getCurrentLocation2D();
    worldToChassis.updateTranslation(chassisPose.getX(), chassisPose.getY(), 0.);
    worldToChassis.updateRotation(0., 0., chassisPose.getOrientation());

    // Chassis to Turret Major
    chassisToTurretMajor.updateRotation(0., 0., turretMajor.getChassisYaw());

#ifdef TARGET_SENTINEL_2026
    turretMajorToTurretWidow.updateRotation(
        0.,
        turretWidow.pitchMotor.getChassisFrameMeasuredAngle().getWrappedValue(),
        turretWidow.yawMotor.getChassisFrameMeasuredAngle().getWrappedValue());

    // World transforms
    worldToTurretMajor = worldToChassis.composeStatic(chassisToTurretMajor);
    worldToVTM = worldToTurretMajor;

    worldToTurretWidow = worldToTurretMajor.composeStatic(turretMajorToTurretWidow);
    turretWidowYawCorrection += turretWidowYawSyncPid.runControllerDerivateError(
        Angle(turretWidowImu.getYaw() + turretWidowYawCorrection)
            .minDifference(worldToTurretWidow.getYaw()),
        0.002f);
    worldToTurretWidow.updateRotation(
        0,
        turretWidowImu.getPitch(),
        turretWidowImu.getYaw() + turretWidowYawCorrection);
    worldToTurretWidow.updateAngularVelocity(0, turretWidowImu.getGy(), turretWidowImu.getGz());
#else
    // Turret Major to Minors
    turretMajorToTurretLeft.updateRotation(
        0.,
        turretLeft.pitchMotor.getChassisFrameMeasuredAngle().getWrappedValue(),
        turretLeft.yawMotor.getChassisFrameMeasuredAngle().getWrappedValue());
    turretMajorToTurretRight.updateRotation(
        0.,
        turretRight.pitchMotor.getChassisFrameMeasuredAngle().getWrappedValue(),
        turretRight.yawMotor.getChassisFrameMeasuredAngle().getWrappedValue());

    // World transforms
    worldToTurretMajor = worldToChassis.composeStatic(chassisToTurretMajor);
    worldToVTM = worldToTurretMajor;

    worldToTurretLeft = worldToTurretMajor.composeStatic(turretMajorToTurretLeft);
    turretLeftYawCorrection += turretLeftYawSyncPid.runControllerDerivateError(
        Angle(turretLeftImu.getYaw() + turretLeftYawCorrection)
            .minDifference(worldToTurretLeft.getYaw()),
        0.002f);
    worldToTurretLeft.updateRotation(
        0,
        turretLeftImu.getPitch(),
        turretLeftImu.getYaw() + turretLeftYawCorrection);
    worldToTurretLeft.updateAngularVelocity(0, turretLeftImu.getGy(), turretLeftImu.getGz());

    worldToTurretRight = worldToTurretMajor.composeStatic(turretMajorToTurretRight);
    turretRightYawCorrection += turretRightYawSyncPid.runControllerDerivateError(
        Angle(turretRightImu.getYaw() + turretRightYawCorrection)
            .minDifference(worldToTurretRight.getYaw()),
        0.002f);
    worldToTurretRight.updateRotation(
        0,
        turretRightImu.getPitch(),
        turretRightImu.getYaw() + turretRightYawCorrection);
    worldToTurretRight.updateAngularVelocity(0, turretRightImu.getGy(), turretRightImu.getGz());
#endif

    // Chassis to Arducam
    chassisToArducam0 = chassisToTurretMajor.composeStatic(MAJOR_TO_ARDUCAM1);
    chassisToArducam1 = chassisToTurretMajor.composeStatic(MAJOR_TO_ARDUCAM2);
    chassisToArducam2 = chassisToTurretMajor.composeStatic(MAJOR_TO_ARDUCAM3);
    chassisToArducam3 = chassisToTurretMajor.composeStatic(MAJOR_TO_ARDUCAM4);
}

}  // namespace aruwsrc::sentry::algorithms::odometry
