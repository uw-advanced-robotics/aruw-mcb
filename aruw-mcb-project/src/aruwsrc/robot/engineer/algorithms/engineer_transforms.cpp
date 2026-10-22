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

#include "aruwsrc/communication/serial/engineer_cv_communication.hpp"
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
    const tap::communication::sensors::imu::AbstractIMU& turretPitchImu,
    const aruwsrc::control::joint::JointSubsystem& extension,
    const aruwsrc::engineer::wrist::WristSubsystem& wrist,
    const aruwsrc::control::joint::JointSubsystem& cubeStorage,
    aruwsrc::communication::serial::EngineerCVCommunication& engineerCVCommunication)
    : chassisOdometry(chassisOdometry),
      chassisImu(chassisImu),
      turret(turret),
      turretPitchImu(turretPitchImu),
      extension(extension),
      wrist(wrist),
      cubeStorage(cubeStorage),
      engineerCVCommunication(engineerCVCommunication),
      worldToChassis(),
      chassisToWorld(),
      chassisToTurretYaw(getHypotheticalChassisToTurretYaw(0)),
      turretYawToTurretPitch(getHypotheticalTurretYawToTurretPitch(0)),
      turretPitchToExtension(getHypotheticalTurretPitchToExtension(0)),
      extensionToWrist(),
      cubeStoreFrameToCubeStoreCenter(),
      worldToTurretPitch(),
      worldToRealsense(),
      worldToReceptacle(),
      worldToEndEffector(),
      cubeStore1ToEndEffector(),
      cubeStore2ToEndEffector(),
      vtmGimbalToEndEffector(),
      endEffectorToCubeDist(),
      COMBeyondTurretPitch(
          {.mass = MASS_BETWEEN_TURRET_PITCH_AND_WRIST_ZERO_EXT.mass + MASS_BEYOND_WRIST.mass,
           .location = Position(0, 0, 0)}),
      COMBeyondWrist(MASS_BEYOND_WRIST),
      worldToReceptacleReceivedTimeMs(-1)
{
}

void EngineerTransforms::updateTransforms()
{
    modm::Location2D chassisPose = chassisOdometry.getCurrentLocation2D();
    // float tpRoll = turretPitchImu.getRoll();
    // float tpPitch = turretPitchImu.getPitch();
    // float tpYaw = turretPitchImu.getYaw();
    float extPos = extension.getPosition();

    // update joint transforms
    worldToChassis.updateTranslation(chassisPose.getX(), chassisPose.getY(), 0.);

    // use odometry yaw because it likely filters more information than imu alone, but only for yaw
    worldToChassis.updateRotation(
        // chassisImu.getRoll(),
        // chassisImu.getPitch(),
        0,
        0,
        chassisPose.getOrientation());
    // worldToChassis.updateAngularVelocity(0., 0., chassisImu.getGz());

    chassisToWorld = worldToChassis.getInverse();

    chassisToTurretYaw.updateRotation(
        0,
        0,
        turret.yawMotor.getChassisFrameMeasuredAngle().getWrappedValue());
    turretYawToTurretPitch.updateRotation(
        0,
        turret.pitchMotor.getChassisFrameMeasuredAngle().getWrappedValue(),
        0);
    turretPitchToExtension = getHypotheticalTurretPitchToExtension(extPos);
    extensionToWrist.updateRotation(wrist.getOrientation());

    cubeStoreFrameToCubeStoreCenter.updateRotation(0, 0, cubeStorage.getPosition());

    Transform turretYawToExtension = turretYawToTurretPitch.composeStatic(turretPitchToExtension);
    Transform extensionToEndEffector = extensionToWrist.composeStatic(WRIST_TO_END_EFFECTOR);
    Transform turretYawToEndEffector = turretYawToExtension.composeStatic(extensionToEndEffector);

    // update requested transforms
    Transform worldToTurretYaw = worldToChassis.composeStatic(chassisToTurretYaw);
    // worldToTurretYaw.updateRotation(
    //     tpRoll,  // could be either inherited or use turret imu, either works
    //     worldToTurretYaw.getPitch(),
    //     tpYaw);

    worldToTurretPitch = worldToTurretYaw.composeStatic(turretYawToTurretPitch);
    // worldToTurretPitch.updateRotation(tpRoll, tpPitch, tpYaw);
    // worldToTurretPitch.updateAngularVelocity(
    //     turretPitchImu.getGx(),
    //     turretPitchImu.getGy(),
    //     turretPitchImu.getGz());

    worldToTurretPitch.updateAngularVelocity(
        0,
        turret.pitchMotor.getChassisFrameVelocity(),
        turret.yawMotor
            .getChassisFrameVelocity());  // should be added to chassis angular velocity, but can't
                                          // use imu rn so need to expose it in odometry

    worldToRealsense = worldToTurretYaw.composeStatic(TURRET_YAW_TO_REALSENSE);
    if (engineerCVCommunication.getIsFresh())
    {
        // cv communication gives the receptacle pose in the camera frame
        worldToReceptacle =
            worldToRealsense.composeStatic(engineerCVCommunication.getCamToReceptacle());
        // Stamp this pose with the packet's received-time; doubles as the validity flag.
        worldToReceptacleReceivedTimeMs = engineerCVCommunication.getLastReceivedTimeMs();
        engineerCVCommunication.markTargetPoseStale();
    }

    Transform cubeStoreCenterToTurretYaw =
        TURRET_YAW_TO_CUBE_STORE_FRAME.composeStatic(cubeStoreFrameToCubeStoreCenter);
    Transform cubeStoreCenterToEndEffector =
        cubeStoreCenterToTurretYaw.composeStatic(turretYawToEndEffector);

    cubeStore1ToEndEffector =
        CUBE_STORE_1_TO_CUBE_STORE_CENTER.composeStatic(cubeStoreCenterToEndEffector);
    cubeStore2ToEndEffector =
        CUBE_STORE_2_TO_CUBE_STORE_CENTER.composeStatic(cubeStoreCenterToEndEffector);

    vtmGimbalToEndEffector = VTM_GIMBAL_TO_EXTENSION.composeStatic(extensionToEndEffector);
    endEffectorToCubeDist =
        turretYawToEndEffector.getInverse().composeStatic(TURRET_YAW_TO_CUBE_DIST);

    worldToEndEffector = worldToTurretYaw.composeStatic(turretYawToEndEffector);

    // COMs
    Transform turretYawToWrist = turretYawToExtension.composeStatic(extensionToWrist);
    Transform worldToWrist = worldToTurretYaw.composeStatic(turretYawToWrist);

    // TODO: tap should have a single operation for this
    COMBeyondWrist.location = worldToWrist.getInverse().apply(MASS_BEYOND_WRIST.location);
    PointMass COMBetweenTurretPitchAndWrist{
        .mass = MASS_BETWEEN_TURRET_PITCH_AND_WRIST_ZERO_EXT.mass,
        .location = worldToTurretPitch.getInverse().apply(
            MASS_BETWEEN_TURRET_PITCH_AND_WRIST_ZERO_EXT.location +
            Vector(extPos * EXT_TO_COM_POS_BETWEEN_TURRET_PITCH_AND_WRIST_SCALAR, 0, 0))};

    COMBeyondTurretPitch = PointMass::merge(COMBetweenTurretPitchAndWrist, COMBeyondWrist);
}

}  // namespace aruwsrc::engineer::algorithms
