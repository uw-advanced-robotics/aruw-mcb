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
#include "aruwsrc/control/turret/turret_subsystem.hpp"

using namespace tap::algorithms;
using namespace tap::algorithms::transforms;

namespace aruwsrc::algorithms::transforms
{

StandardTransformer::StandardTransformer(
    const tap::communication::sensors::imu::mpu6500::Mpu6500& chassisImu
    ) :  
      worldToChassisIMUTransform(
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL),
      TurretIMUToCameraTransform(
          0., TURRETIMU_TO_CAMERA_Y_OFFSET, 0., 0., 0., 0.),
      turretIMUToGunTransform(
          0.,
          TURRETIMU_TO_GUN_Y_OFFSET,
          TURRETIMU_TO_GUN_Z_OFFSET,
          0.,
          0.,
          0.),

      // Transforms that are compositions
      worldToTurretIMUTransform(
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL),
      worldToChassisTransform(
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL),

      // Transforms that are inverses
      chassisToWorldTransform(
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL),
      turretIMUToChassisTransform(
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL),
      cameraToTurretIMUTransform(
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL),

      // Constant transforms
      chassisToTurretIMUTransform(
        0., 0., CHASSIS_TO_TURRET_Z_OFFSET, 0., 0., 0.),
      chassisIMUToChassisTransform(
          CHASSISIMU_TO_CHASSIS_X_OFFSET,
          0.,
          CHASSISIMU_TO_CHASSIS_Z_OFFSET,
          0.,
          0.,
          0.),
      chassisImu(chassisImu),
      kf(KF_A, KF_C, KF_Q, KF_R, KF_P0)
{
    // reference https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html
    // reference disagrees with the forward kinematics.. (in terms of signedness)
    // lol chat gpt also disagrees
    // Forward kinematic matrix for mecanum drive

    // after some big brain simulation, this is the matrix that
    // is produced from the referenced matrix IF all right motors
    // have their velocity multiplied by -1
    // it makes sense that the reading would need to be multiplied
    // by -1 *somewhere*, but why here? couldn't that
    // have been handled in isInverted upon motor instantiation?
    wheelVelToChassisVelMat[X][LF] = 1;
    wheelVelToChassisVelMat[X][RF] = -1;
    wheelVelToChassisVelMat[X][LB] = 1;
    wheelVelToChassisVelMat[X][RB] = -1;
    wheelVelToChassisVelMat[Y][LF] = -1;
    wheelVelToChassisVelMat[Y][RF] = -1;
    wheelVelToChassisVelMat[Y][LB] = 1;
    wheelVelToChassisVelMat[Y][RB] = 1;

    // angular velocity (double check this part)
    wheelVelToChassisVelMat[R][LF] = -1.0 / chassis::WHEELBASE_HYPOTENUSE;
    wheelVelToChassisVelMat[R][RF] = -1.0 / chassis::WHEELBASE_HYPOTENUSE;
    wheelVelToChassisVelMat[R][LB] = -1.0 / chassis::WHEELBASE_HYPOTENUSE;
    wheelVelToChassisVelMat[R][RB] = -1.0 / chassis::WHEELBASE_HYPOTENUSE;
    wheelVelToChassisVelMat *= (chassis::WHEEL_RADIUS / 4);
}

void StandardTransformer::update()
{
    updateOdometry();
    // updateTransforms();
}

void StandardTransformer::init(
    const chassis::MecanumChassisSubsystem* chassisSubsystem,
    const aruwsrc::control::turret::StandardTurretSubsystem* turretSubsystem
    )
{
    float initialKFVals[9] = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
    this->kf.init(initialKFVals);

    this->chassis = chassisSubsystem;
    this->turret = turretSubsystem;
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

const Transform<ChassisFrame, WorldFrame>& StandardTransformer::getChassisToWorldTransform()
{
    worldToChassisTransform = compose<WorldFrame, ChassisIMUFrame, ChassisFrame>(
        worldToChassisIMUTransform,
        chassisIMUToChassisTransform);

    chassisToWorldTransform = worldToChassisTransform.getInverse();
    return chassisToWorldTransform;
}

const Transform<TurretIMUFrame, ChassisFrame>& StandardTransformer::getTurretIMUToChassisTransform()
{
    turretIMUToChassisTransform = chassisToTurretIMUTransform.getInverse();
    return turretIMUToChassisTransform;
}

const Transform<CameraFrame, TurretIMUFrame>& StandardTransformer::getCameraToTurretIMUTransform()
{
    cameraToTurretIMUTransform = TurretIMUToCameraTransform.getInverse();
    return cameraToTurretIMUTransform;
}

void StandardTransformer::updateTransforms()
{
    // update all transforms that can't be derived from others
    worldToChassisIMUTransform = Transform<WorldFrame, ChassisIMUFrame>(
        -chassisWorldPosition.getX(),
        -chassisWorldPosition.getY(),
        -chassisWorldPosition.getZ(),
        -chassisWorldOrientation.getX(),
        -chassisWorldOrientation.getY(),
        -chassisWorldOrientation.getZ());

    TurretIMUToCameraTransform = Transform<TurretIMUFrame, CameraFrame>(
        0.,
        TURRETIMU_TO_CAMERA_Y_OFFSET,
        0.,
        turretWorldOrientation.getX(),
        turretWorldOrientation.getY(),
        turretWorldOrientation.getZ());

    turretIMUToGunTransform = Transform<TurretIMUFrame, GunFrame>(
        0.,
        TURRETIMU_TO_GUN_Y_OFFSET,
        TURRETIMU_TO_GUN_Z_OFFSET,
        turretWorldOrientation.getX(),
        turretWorldOrientation.getY(),
        turretWorldOrientation.getZ());
}

void StandardTransformer::updateOdometry()
{
    // nasty to return an array in c++, so do this for now
    float nextKFInput[int(OdomInput::NUM_INPUTS)] = {};
    fillKFInput(nextKFInput);

    kf.performUpdate(nextKFInput);

    updateInternalOdomFromKF();
}

void StandardTransformer::fillKFInput(float nextKFInput[])
{
    modm::Matrix<float, 3, 1> chassisVelocity = getVelocityChassisRelative();
    rotateChassisVectorToWorld(chassisVelocity);

    modm::Matrix<float, 3, 1> chassisAcceleration = getAccelerationChassisRelative();
    rotateChassisVectorToWorld(chassisAcceleration);

    nextKFInput[int(OdomInput::VEL_X)] = chassisVelocity[0][0];
    nextKFInput[int(OdomInput::VEL_Y)] = chassisVelocity[1][0];
    nextKFInput[int(OdomInput::VEL_Z)] = chassisVelocity[2][0];

    nextKFInput[int(OdomInput::ACC_X)] = chassisAcceleration[0][0];
    nextKFInput[int(OdomInput::ACC_Y)] = chassisAcceleration[1][0];
    nextKFInput[int(OdomInput::ACC_Z)] = chassisAcceleration[2][0];
}

void StandardTransformer::updateInternalOdomFromKF()
{
    const auto& state = kf.getStateVectorAsMatrix();

    // update the store odometry for easy access internally
    chassisWorldPosition.setX(state[int(OdomState::POS_X)]);
    chassisWorldPosition.setY(state[int(OdomState::POS_Y)]);
    chassisWorldPosition.setZ(state[int(OdomState::POS_Z)]);

    chassisWorldOrientation.setX(chassisImu.getRoll());
    chassisWorldOrientation.setY(chassisImu.getPitch());
    chassisWorldOrientation.setZ(chassisImu.getYaw());

    // we cannot query turret roll (for now)
    turretWorldOrientation.setX(0);
    turretWorldOrientation.setY(turretMCB.getPitch());
    turretWorldOrientation.setZ(turretMCB.getYaw());
}

// void StandardTransformer::getVelocityChassisRelative(modm::Matrix<float, 3, 1>& cV)
modm::Matrix<float, 3, 1> StandardTransformer::getVelocityChassisRelative()
{
    if (!areMotorsOnline()) 
        return modm::Matrix<float, 3, 1>().zeroMatrix();

    modm::Matrix<float, 4, 1> wheelVelocity;
    wheelVelocity[LF][0] = leftFrontMotor->getShaftRPM();
    wheelVelocity[RF][0] = rightFrontMotor->getShaftRPM();
    wheelVelocity[LB][0] = leftBackMotor->getShaftRPM();
    wheelVelocity[RB][0] = rightBackMotor->getShaftRPM();
    return (wheelVelToChassisVelMat * convertRawRPM(wheelVelocity));
}

bool StandardTransformer::areMotorsOnline()
{
    // motors aren't registered
    if (leftBackMotor == nullptr) return false;

    return leftBackMotor->isMotorOnline() && rightBackMotor->isMotorOnline() &&
           leftFrontMotor->isMotorOnline() && rightFrontMotor->isMotorOnline();
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
    // but for now I'm lazy...

    // need to get a CMSISMat from chassisRelVector since Transforms take
    // CMSISMats :(

    float data[3] = {chassisRelVector[0][0], chassisRelVector[1][0], chassisRelVector[2][0]};
    CMSISMat<3, 1> cmsisChassisRelVector = CMSISMat<3, 1>(data);

    // Transform.applyToVector only rotates, doesn't translate
    CMSISMat<3, 1> cmsisWorldRelVector =
        worldToChassisIMUTransform.getInverse().applyToVector(cmsisChassisRelVector);

    // copy transformed positions back to original vector
    chassisRelVector[0][0] = cmsisWorldRelVector.data[0];
    chassisRelVector[1][0] = cmsisWorldRelVector.data[1];
    chassisRelVector[2][0] = cmsisWorldRelVector.data[2];
}
}  // namespace aruwsrc::algorithms::transforms
