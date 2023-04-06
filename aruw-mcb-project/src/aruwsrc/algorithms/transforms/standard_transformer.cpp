/*
 * Copyright (c) 2021-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/algorithms/transforms/standard_transformer.hpp"

#include "tap/algorithms/transforms/transform.hpp"
#include "tap/algorithms/transforms/transformer.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"

#include "aruwsrc/algorithms/transforms/standard_frames.hpp"

#include "aruwsrc/robot/standard/standard_turret_subsystem.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "modm/math/geometry/angle.hpp"

using namespace tap::algorithms;
using namespace tap::algorithms::transforms;

namespace aruwsrc::algorithms::transforms
{

StandardTransformer::StandardTransformer(
    tap::communication::sensors::imu::mpu6500::Mpu6500& chassisImu
    ) :  
      chassisImu(chassisImu),
      posKf(POS_KF_A, POS_KF_C, POS_KF_Q, POS_KF_R, POS_KF_P0),
      rotKf(ROT_KF_A, ROT_KF_C, ROT_KF_Q, ROT_KF_R, ROT_KF_P0)
{ }

void StandardTransformer::update()
{
    updateOdometry();
    updateTransforms();
}

void StandardTransformer::updateOdometry()
{
    float nextPosKFInput[int(PosOdomInput::NUM_INPUTS)] = {};
    float nextRotKFInput[int(RotOdomInput::NUM_INPUTS)] = {};
    fillPosKFInput(nextPosKFInput);
    fillRotKFInput(nextRotKFInput);
    posKf.performUpdate(nextPosKFInput);
    rotKf.performUpdate(nextRotKFInput);
    updateInternalOdomFromKF();
}

void StandardTransformer::updateTransforms()
{
    // update all transforms that can't be derived from others
    float worldToChassisPosition[3] = {
        -chassisWorldPosition.getX(),
        -chassisWorldPosition.getY(),
        -chassisWorldPosition.getZ()};
    CMSISMat<3,1> cmsisWorldToChassisPosition(worldToChassisPosition);
    
    worldToChassisIMUTransform.updatePosition(cmsisWorldToChassisPosition);
    worldToChassisIMUTransform.updateRotation(
        -chassisWorldOrientation.getX(),
        -chassisWorldOrientation.getY(),
        -chassisWorldOrientation.getZ());

    float turretIMUTOCameraPosition[3] = {
        0.,
        TURRETIMU_TO_CAMERA_Y_OFFSET,
        0.};
    CMSISMat<3,1> cmsisTurretIMUToCameraPosition(turretIMUTOCameraPosition);
    
    turretIMUToCameraTransform.updatePosition(cmsisTurretIMUToCameraPosition);
    turretIMUToCameraTransform.updateRotation(
        turretWorldOrientation.getX(),
        turretWorldOrientation.getY(),
        turretWorldOrientation.getZ());

    float turretIMUToGunPos[3] = {
        0.,
        TURRETIMU_TO_GUN_Y_OFFSET,
        TURRETIMU_TO_GUN_Z_OFFSET};
    CMSISMat<3,1> cmsisTurretIMUToGunPos(turretIMUToGunPos);
    
    turretIMUToGunTransform.updatePosition(cmsisTurretIMUToGunPos);
    turretIMUToGunTransform.updateRotation(
        turretWorldOrientation.getX(),
        turretWorldOrientation.getY(),
        turretWorldOrientation.getZ());
}

void StandardTransformer::init(
    const chassis::MecanumChassisSubsystem* chassisSubsystem,
    const aruwsrc::control::turret::StandardTurretSubsystem* turretSubsystem)
{
    this->chassis = chassisSubsystem;
    this->turret = turretSubsystem;

    float initialPosKFVals[int(PosOdomState::NUM_STATES)] = {};
    float initialRotKFVals[int(RotOdomState::NUM_STATES)] = {};
    this->posKf.init(initialPosKFVals);
    this->rotKf.init(initialRotKFVals);

    resetTransforms();
}

const Transform<WorldFrame, ChassisFrame>& StandardTransformer::getWorldToChassisTransform()
{
    worldToChassisTransform = compose<WorldFrame, ChassisIMUFrame, ChassisFrame>(
        worldToChassisIMUTransform,
        chassisIMUToChassisTransform);

    return worldToChassisTransform;
}

const Transform<WorldFrame, TurretIMUFrame>& StandardTransformer::getWorldToTurretIMUTransform()
{
    worldToChassisTransform = compose<WorldFrame, ChassisIMUFrame, ChassisFrame>(
        worldToChassisIMUTransform,
        chassisIMUToChassisTransform);

    worldToTurretIMUTransform = compose<WorldFrame, ChassisFrame, TurretIMUFrame>(
        worldToChassisTransform,
        chassisToTurretIMUTransform);

    return worldToTurretIMUTransform;
}

const Transform<ChassisFrame, TurretIMUFrame>& StandardTransformer::getChassisToTurretIMUTransform()
{
    return chassisToTurretIMUTransform;
}

const Transform<ChassisIMUFrame, ChassisFrame>& StandardTransformer::getChassisIMUToChassisTransform() 
{
    return chassisIMUToChassisTransform;
}

const Transform<TurretIMUFrame, CameraFrame>& StandardTransformer::getTurretIMUToCameraTransform()
{
    return turretIMUToCameraTransform;
}

const Transform<TurretIMUFrame, GunFrame>& StandardTransformer::getTurretIMUToGunTransform()
{
    return turretIMUToGunTransform;
}

void StandardTransformer::fillPosKFInput(float nextKFInput[])
{
    // retrieves <vz, vy, yaw_velocity>
    modm::Matrix<float, 3, 1> chassisPlanarVelocity = chassis->getActualVelocityChassisRelative();
    
    // relative to chassis, so velocity must be 0
    float xyzVelocityData[3] = {chassisPlanarVelocity[0][0], chassisPlanarVelocity[1][0], 0.f};
    modm::Matrix<float, 3, 1> chassisVelocity = modm::Matrix<float, 3, 1>(xyzVelocityData);
    rotateChassisVectorToWorld(chassisVelocity);

    modm::Matrix<float, 3, 1> chassisAcceleration = getAccelerationChassisRelative();
    rotateChassisVectorToWorld(chassisAcceleration);

    nextKFInput[int(PosOdomInput::VEL_X)] = chassisVelocity[0][0];
    nextKFInput[int(PosOdomInput::VEL_Y)] = chassisVelocity[1][0];
    nextKFInput[int(PosOdomInput::VEL_Z)] = chassisVelocity[2][0];

    nextKFInput[int(PosOdomInput::ACC_X)] = chassisAcceleration[0][0];
    nextKFInput[int(PosOdomInput::ACC_Y)] = chassisAcceleration[1][0];
    nextKFInput[int(PosOdomInput::ACC_Z)] = chassisAcceleration[2][0];
}

void StandardTransformer::fillRotKFInput(float nextKFInput[])
{
    // retrieves <vz, vy, yaw_velocity>
    modm::Matrix<float, 3, 1> chassisPlanarVelocity = chassis->getActualVelocityChassisRelative();
    
    // radians per second
    float yawAngularVelocity = chassisPlanarVelocity[2][0];
    nextKFInput[int(RotOdomInput::VEL_YAW)] = yawAngularVelocity;

    // note: this will frick up the kalman filter when we cross 2pi => 0 or 0 =? 2pi
    nextKFInput[int(RotOdomInput::POS_YAW)] =  modm::toRadian(chassisImu.getYaw());
    // TODO: unwrap this later, for now observe what it looks like when it wraps
    // (how quickly it adjusts) 
}

void StandardTransformer::updateInternalOdomFromKF()
{
    const auto& posState = posKf.getStateVectorAsMatrix();
    const auto& rotState = rotKf.getStateVectorAsMatrix();
    const aruwsrc::can::TurretMCBCanComm* turretMCB = turret->getTurretMCB();

    // update the store odometry for easy access internally
    chassisWorldPosition.setX(posState[int(PosOdomState::POS_X)]);
    chassisWorldPosition.setY(posState[int(PosOdomState::POS_Y)]);
    chassisWorldPosition.setZ(posState[int(PosOdomState::POS_Z)]);

    chassisWorldOrientation.setX(rotState[int(RotOdomState::POS_YAW)]);
    chassisWorldOrientation.setY(0);
    chassisWorldOrientation.setZ(0);

    // we cannot query turret roll (for now)
    turretWorldOrientation.setX(0);
    turretWorldOrientation.setY(turretMCB->getPitch());
    turretWorldOrientation.setZ(turretMCB->getYaw());
}

void StandardTransformer::resetTransforms() {
    // set all transforms to be identity
    setIdentityTransform(worldToChassisIMUTransform);
    setIdentityTransform(turretIMUToCameraTransform);
    setIdentityTransform(turretIMUToGunTransform);

    setIdentityTransform(worldToTurretIMUTransform);
    setIdentityTransform(worldToChassisTransform);

    setIdentityTransform(chassisToTurretIMUTransform);
    setIdentityTransform(chassisIMUToChassisTransform);

    initializeStaticTransforms();
}

void StandardTransformer::initializeStaticTransforms() {
    float turretIMUToCameraTransformPos[3] = {0.f, TURRETIMU_TO_CAMERA_Y_OFFSET, 0.f};
    CMSISMat<3,1> cmsisTurretIMUToCameraDefaultPos(turretIMUToCameraTransformPos);
    turretIMUToCameraTransform.updatePosition(cmsisTurretIMUToCameraDefaultPos);

    float turretIMUToGunDefaultPos[3] = {0.f, TURRETIMU_TO_GUN_Y_OFFSET, TURRETIMU_TO_GUN_Z_OFFSET};
    CMSISMat<3,1> cmsisTurretIMUToGunDefaultPos(turretIMUToGunDefaultPos);
    turretIMUToGunTransform.updatePosition(cmsisTurretIMUToGunDefaultPos);
    
    float chassisToTurretIMUTransformPos[3] = {0., 0., CHASSIS_TO_TURRET_Z_OFFSET};
    CMSISMat<3,1> cmsisChassisToTurretIMUTransformPos(chassisToTurretIMUTransformPos);
    chassisToTurretIMUTransform.updatePosition(cmsisChassisToTurretIMUTransformPos);

    float chassisIMUToChassisTransformPos[3] = {CHASSISIMU_TO_CHASSIS_X_OFFSET, 0., CHASSISIMU_TO_CHASSIS_Z_OFFSET};
    CMSISMat<3,1> cmsisChassisIMUToChassisTransformPos(chassisIMUToChassisTransformPos);
    chassisIMUToChassisTransform.updatePosition(cmsisChassisIMUToChassisTransformPos);
}

modm::Matrix<float, 3, 1> StandardTransformer::getAccelerationChassisRelative()
{
    // for this code to be running the imu has to be on (since the mcb is on)
    float accData[3] = {chassisImu.getAx(), chassisImu.getAy(), chassisImu.getAz()};
    return modm::Matrix<float, 3, 1>(accData);
}

void StandardTransformer::rotateChassisVectorToWorld(modm::Matrix<float, 3, 1>& chassisRelVector)
{
    // our chassisToWorldTransform is 1 cycle out of date, so in the future we
    // may want to extrapolate values to construct the up-to-date transform
    float data[3] = {chassisRelVector[0][0], chassisRelVector[1][0], chassisRelVector[2][0]};
    CMSISMat<3, 1> cmsisChassisRelVector = CMSISMat<3, 1>(data);

    CMSISMat<3, 1> cmsisWorldRelVector =
        worldToChassisIMUTransform.getInverse().applyToVector(cmsisChassisRelVector);

    chassisRelVector[0][0] = cmsisWorldRelVector.data[0];
    chassisRelVector[1][0] = cmsisWorldRelVector.data[1];
    chassisRelVector[2][0] = cmsisWorldRelVector.data[2];
}
}  // namespace aruwsrc::algorithms::transforms
