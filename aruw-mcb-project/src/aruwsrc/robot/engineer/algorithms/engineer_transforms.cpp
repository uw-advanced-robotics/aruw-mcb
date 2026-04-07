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

#include "engineer_transforms.hpp"

#include "tap/communication/sensors/imu/abstract_imu.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "aruwsrc/control/joint/joint_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_subsystem.hpp"

using namespace tap::algorithms::odometry;
using namespace tap::algorithms::transforms;

namespace aruwsrc::engineer::algorithms
{
EngineerTransforms::EngineerTransforms(
    const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry,
    const tap::communication::sensors::imu::AbstractIMU& chassisImu,
    const aruwsrc::control::turret::TurretSubsystem& turret,
    const tap::communication::sensors::imu::ImuInterface& turretPitchImu,
    const aruwsrc::control::joint::JointSubsystem& extension,
    const aruwsrc::engineer::wrist::WristSubsystem& wrist,
    const aruwsrc::control::joint::JointSubsystem& roll,
    const aruwsrc::control::joint::JointSubsystem& cubeStorage)
    : chassisOdometry(chassisOdometry),
      chassisImu(chassisImu),
      turret(turret),
      turretPitchImu(turretPitchImu),
      extension(extension),
      wrist(wrist),
      roll(roll),
      cubeStorage(cubeStorage),
      worldToChassis(Transform::identity()),
      chassisToTurretYaw(getHypotheticalChassisToTurretYaw(0)),
      turretYawToTurretPitch(getHypotheticalTurretYawToTurretPitch(0)),
      turretPitchToExtension(getHypotheticalTurretPitchToExtension(0)),
      extensionToWrist(Transform::identity()),
      wristToWristRoll(Transform::identity()),
      cubeStoreFrameToCubeStoreCenter(Transform::identity()),
      worldToTurretPitch(Transform::identity()),
      worldToRealsense(Transform::identity()),
      worldToEndEffector(Transform::identity()),
      cubeStore1ToEndEffector(Transform::identity()),
      cubeStore2ToEndEffector(Transform::identity()),
      vtmGimbalToEndEffector(Transform::identity()),
      endEffectorToCubeDist(Transform::identity()),
      COMBeyondTurretPitch(
          {.mass = MASS_BETWEEN_TURRET_PITCH_AND_WRIST_ZERO_EXT.mass + MASS_BEYOND_WRIST_ROLL.mass,
           .location = Position(0, 0, 0)}),
      COMBeyondWristRoll(MASS_BEYOND_WRIST_ROLL)
{
}

void EngineerTransforms::updateTransforms()
{
    // update joint transforms
    modm::Location2D chassisPose = chassisOdometry.getCurrentLocation2D();
    worldToChassis.updateTranslation(chassisPose.getX(), chassisPose.getY(), 0.);
    // use odometry yaw because it likely filters more information than imu alone, but only for yaw
    worldToChassis.updateRotation(
        chassisImu.getRoll(),
        chassisImu.getPitch(),
        chassisPose.getOrientation());
    // worldToChassis.updateAngularVelocity(0., 0., chassisImu.getGz());

    chassisToTurretYaw.updateRotation(
        0,
        0,
        turret.yawMotor.getChassisFrameMeasuredAngle().getWrappedValue());
    turretYawToTurretPitch.updateRotation(0, turretPitchImu.getPitch(), 0);
    turretPitchToExtension = getHypotheticalTurretPitchToExtension(extension.getPosition());
    extensionToWrist.updateRotation(
        wrist.computeWristOrientation(0, 0).getRotation());  // TODO: update
    wristToWristRoll.updateRotation(roll.getPosition(), 0, 0);

    cubeStoreFrameToCubeStoreCenter.updateRotation(0, 0, cubeStorage.getPosition());

    // update requested transforms

    Transform worldToTurretYaw = worldToChassis.composeStatic(chassisToTurretYaw);
    worldToTurretYaw.updateRotation(
        turretPitchImu.getRoll(),  // could be either inherited or use turret imu, either works
        worldToTurretYaw.getPitch(),
        turretPitchImu.getYaw());

    worldToTurretPitch = worldToTurretYaw.composeStatic(turretYawToTurretPitch);
    worldToTurretPitch.updateRotation(0, turretPitchImu.getPitch(), turretPitchImu.getYaw());
    worldToTurretPitch.updateAngularVelocity(0, turretPitchImu.getGy(), turretPitchImu.getGz());

    worldToRealsense = worldToTurretYaw.composeStatic(TURRET_YAW_TO_REALSENSE);

    Transform cubeStoreCenterToTurretYaw =
        TURRET_YAW_TO_CUBE_STORE_FRAME.composeStatic(cubeStoreFrameToCubeStoreCenter);
    Transform extensionToEndEffector =
        extensionToWrist.composeStatic(wristToWristRoll).composeStatic(WRIST_ROLL_TO_END_EFFECTOR);
    Transform turretYawToEndEffector = turretYawToTurretPitch.composeStatic(turretPitchToExtension)
                                           .composeStatic(extensionToEndEffector);
    cubeStore1ToEndEffector =
        CUBE_STORE_1_TO_CUBE_STORE_CENTER.composeStatic(cubeStoreCenterToTurretYaw)
            .composeStatic(turretYawToEndEffector);
    cubeStore2ToEndEffector =
        CUBE_STORE_2_TO_CUBE_STORE_CENTER.composeStatic(cubeStoreCenterToTurretYaw)
            .composeStatic(turretYawToEndEffector);

    vtmGimbalToEndEffector = VTM_GIMBAL_TO_EXTENSION.composeStatic(extensionToEndEffector);
    endEffectorToCubeDist =
        turretYawToEndEffector.getInverse().composeStatic(TURRET_YAW_TO_CUBE_DIST);

    worldToEndEffector = worldToTurretYaw.composeStatic(turretYawToEndEffector);

    // COMs
    // TODO: optimize redundancies
    Transform worldToWristRoll = worldToTurretPitch.composeStatic(turretPitchToExtension)
                                     .composeStatic(extensionToWrist)
                                     .composeStatic(wristToWristRoll);

    // TODO: tap should have a single operation for this
    COMBeyondWristRoll.location =
        worldToWristRoll.getInverse().apply(MASS_BEYOND_WRIST_ROLL.location);
    PointMass COMBetweenTurretPitchAndWrist{
        .mass = MASS_BETWEEN_TURRET_PITCH_AND_WRIST_ZERO_EXT.mass,
        .location = worldToTurretPitch.getInverse().apply(
            MASS_BETWEEN_TURRET_PITCH_AND_WRIST_ZERO_EXT.location +
            Vector(
                extension.getPosition() * EXT_TO_COM_POS_BETWEEN_TURRET_PITCH_AND_WRIST_SCALAR,
                0,
                0))};

    COMBeyondTurretPitch = PointMass::merge(COMBetweenTurretPitchAndWrist, COMBeyondWristRoll);
}

}  // namespace aruwsrc::engineer::algorithms
