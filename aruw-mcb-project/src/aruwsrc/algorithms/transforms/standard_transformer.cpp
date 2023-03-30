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

// #include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
// #include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"

using namespace tap::algorithms;
using namespace tap::algorithms::transforms;
// using namespace aruwsrc::chassis;

namespace aruwsrc::algorithms::transforms
{

StandardTransformer::StandardTransformer(
    tap::communication::sensors::imu::mpu6500::Mpu6500& chassisImu,
    aruwsrc::can::TurretMCBCanComm& turretMCB)
    :  // TODO: store transforms in an array so we don't have to initialize them here (very ugly !!!
       // !! ! !) Transforms that are dynamically updated
      worldToChassisIMUTransform(Transform<WorldFrame, ChassisIMUFrame>(
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL)),
      TurretIMUToCameraTransform(
          Transform<TurretIMUFrame, CameraFrame>(0., TURRETIMU_TO_CAMERA_Y_OFFSET, 0., 0., 0., 0.)),
      turretIMUToGunTransform(Transform<TurretIMUFrame, GunFrame>(
          0.,
          TURRETIMU_TO_GUN_Y_OFFSET,
          TURRETIMU_TO_GUN_Z_OFFSET,
          0.,
          0.,
          0.)),

      // Transforms that are compositions
      worldToTurretIMUTransform(Transform<WorldFrame, TurretIMUFrame>(
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL)),
      worldToChassisTransform(Transform<WorldFrame, ChassisFrame>(
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL)),

      // Transforms that are inverses
      chassisToWorldTransform(Transform<ChassisFrame, WorldFrame>(
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL)),
      turretIMUToChassisTransform(Transform<TurretIMUFrame, ChassisFrame>(
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL)),
      cameraToTurretIMUTransform(Transform<CameraFrame, TurretIMUFrame>(
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL,
          TRANSFORM_PLACEHOLDER_VAL)),

      // Constant transforms
      chassisToTurretIMUTransform(
          Transform<ChassisFrame, TurretIMUFrame>(0., 0., CHASSIS_TO_TURRET_Z_OFFSET, 0., 0., 0.)),
      chassisIMUToChassisTransform(Transform<ChassisIMUFrame, ChassisFrame>(
          CHASSISIMU_TO_CHASSIS_X_OFFSET,
          0.,
          CHASSISIMU_TO_CHASSIS_Z_OFFSET,
          0.,
          0.,
          0.)),

      chassisImu(chassisImu),
      turretMCB(turretMCB),
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


    std::cerr << "world to chassis rotation " << std::endl;
    std::cerr << worldToChassisIMUTransform.rotation.data[0] << std::endl;
    std::cerr << worldToChassisIMUTransform.rotation.data[1] << std::endl;
    std::cerr << worldToChassisIMUTransform.rotation.data[2] << std::endl;

}

void StandardTransformer::update()
{
    updateOdometry();
    // testing odometry without rotation: don't update transforms
    // updateTransforms();
}

void StandardTransformer::init(
    const tap::motor::DjiMotor* rightFrontMotor,
    const tap::motor::DjiMotor* leftFrontMotor,
    const tap::motor::DjiMotor* rightBackMotor,
    const tap::motor::DjiMotor* leftBackMotor)
{
    float initialKFVals[9] = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
    this->kf.init(initialKFVals);

    this->rightFrontMotor = rightFrontMotor;
    this->leftFrontMotor = leftFrontMotor;
    this->rightBackMotor = rightBackMotor;
    this->leftBackMotor = leftBackMotor;
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
    // float nextKFInput[int(OdomInput::NUM_INPUTS)] = {};
    fillKFInput(nextKFInput);

    kf.performUpdate(nextKFInput);

    updateInternalOdomFromKF();
}

void StandardTransformer::fillKFInput(float nextKFInput[])
{
    // modm::Matrix<float, 3, 1> chassisVelocity = getVelocityChassisRelative();
    // chassisVelocity = getVelocityChassisRelative(chassisVelocity);
    getVelocityChassisRelative(chassisVelocity);

    rotateChassisVectorToWorld(chassisVelocity);

    modm::Matrix<float, 3, 1> chassisAcceleration = getAccelerationChassisRelative();
    rotateChassisVectorToWorld(chassisAcceleration);


    nextKFInput[int(OdomInput::VEL_X)] = chassisVelocity[0][0];
    nextKFInput[int(OdomInput::VEL_Y)] = chassisVelocity[1][0];
    nextKFInput[int(OdomInput::VEL_Z)] = chassisVelocity[2][0];

    // TODO: for testing only, remove
    vel_x_in = nextKFInput[int(OdomInput::VEL_X)];

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

void StandardTransformer::getVelocityChassisRelative(modm::Matrix<float, 3, 1>& cV)
// modm::Matrix<float, 3, 1> StandardTransformer::getVelocityChassisRelative(modm::Matrix<float, 3, 1>& chassisVelocity)
{
    if (!areMotorsOnline()) cV = modm::Matrix<float, 3, 1>().zeroMatrix();
    else {
        wheelVelocity[LF][0] = leftFrontMotor->getShaftRPM();
        wheelVelocity[RF][0] = rightFrontMotor->getShaftRPM();
        wheelVelocity[LB][0] = leftBackMotor->getShaftRPM();
        wheelVelocity[RB][0] = rightBackMotor->getShaftRPM();
        cV = (wheelVelToChassisVelMat * convertRawRPM(wheelVelocity));
    }

    // modm::Matrix<float, WheelRPMIndex::NUM_MOTORS, 1> wheelVelocity;
    // modm::Matrix<float, WheelRPMIndex::NUM_MOTORS, 1> wheelVelocity;

    // wheelVelocity[LF][0] = leftFrontMotor->getShaftRPM();
    // wheelVelocity[RF][0] = rightFrontMotor->getShaftRPM();
    // wheelVelocity[LB][0] = leftBackMotor->getShaftRPM();
    // wheelVelocity[RB][0] = rightBackMotor->getShaftRPM();

    // chassisVelocityInGetVelFN = wheelVelToChassisVelMat * convertRawRPM(wheelVelocity);
    // return chassisVelocityInGetVelFN;

    // chassisVelocity = (wheelVelToChassisVelMat * convertRawRPM(wheelVelocity));
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
    
    // std::cerr << "rotate cmsis input " << std::endl;
    // std::cerr << cmsisChassisRelVector.data[0] << std::endl;
    // std::cerr << cmsisChassisRelVector.data[1] << std::endl;
    // std::cerr << cmsisChassisRelVector.data[2] << std::endl;

    // std::cerr << "world to chassis rotation " << std::endl;
    // std::cerr << worldToChassisIMUTransform.rotation.data[0] << std::endl;
    // std::cerr << worldToChassisIMUTransform.rotation.data[1] << std::endl;
    // std::cerr << worldToChassisIMUTransform.rotation.data[2] << std::endl;
    // std::cerr << cmsisChassisRelVector.data[1] << std::endl;
    // std::cerr << cmsisChassisRelVector.data[2] << std::endl;

    // Transform.applyToVector only rotates, doesn't translate
    CMSISMat<3, 1> cmsisWorldRelVector =
        worldToChassisIMUTransform.getInverse().applyToVector(cmsisChassisRelVector);

    // std::cerr << "rotate cmsis output" << std::endl;
    // std::cerr << cmsisWorldRelVector.data[0] << std::endl;
    // std::cerr << cmsisWorldRelVector.data[1] << std::endl;
    // std::cerr << cmsisWorldRelVector.data[2] << std::endl;

    // copy transformed positions back to original vector
    chassisRelVector[0][0] = cmsisWorldRelVector.data[0];
    chassisRelVector[1][0] = cmsisWorldRelVector.data[1];
    chassisRelVector[2][0] = cmsisWorldRelVector.data[2];
}
}  // namespace aruwsrc::algorithms::transforms
