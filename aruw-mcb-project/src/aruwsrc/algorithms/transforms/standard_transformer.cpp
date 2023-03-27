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
#include "tap/algorithms/transforms/transformer.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "aruwsrc/algorithms/transforms/standard_frames.hpp"

#include "tap/control/chassis/chassis_subsystem_interface.hpp"

// #include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"

#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"

using namespace tap::algorithms;
using namespace tap::algorithms::transforms;
// using namespace aruwsrc::chassis;

namespace aruwsrc::algorithms::transforms {

    StandardTransformer::StandardTransformer
    (
        tap::communication::sensors::imu::mpu6500::Mpu6500& chassisImu,
        aruwsrc::can::TurretMCBCanComm& turretMCB
    )
    :
    // Transforms that are dynamically updated
    worldToChassisIMUTransform(Transform<WorldFrame, ChassisIMUFrame>(TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL)),
    TurretIMUToCameraTransform(Transform<TurretIMUFrame, CameraFrame>(0., TURRETIMU_TO_CAMERA_Y_OFFSET, 0., 0. ,0., 0.)),
    turretIMUToGunTransform(Transform<TurretIMUFrame, GunFrame> (0., TURRETIMU_TO_GUN_Y_OFFSET, TURRETIMU_TO_GUN_Z_OFFSET, 0., 0., 0.)),

    // Transforms that are compositions
    worldToTurretIMUTransform(Transform<WorldFrame, TurretIMUFrame>(TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL)),
    worldToChassisTransform(Transform<WorldFrame, ChassisFrame>(TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL)),

    // Transforms that are inverses
    chassisToWorldTransform(Transform<ChassisFrame, WorldFrame>(TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL)),
    turretIMUToChassisTransform(Transform<TurretIMUFrame, ChassisFrame>(TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL)),
    cameraToTurretIMUTransform(Transform<CameraFrame, TurretIMUFrame>(TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL, TRANSFORM_PLACEHOLDER_VAL)),

    // Constant transforms
    chassisToTurretIMUTransform(Transform<ChassisFrame,TurretIMUFrame>(0., 0., CHASSIS_TO_TURRET_Z_OFFSET, 0., 0., 0.)),
    chassisIMUToChassisTransform(Transform<ChassisIMUFrame, ChassisFrame> (CHASSISIMU_TO_CHASSIS_X_OFFSET, 0., CHASSISIMU_TO_CHASSIS_Z_OFFSET, 0., 0., 0.)),

    chassisImu(chassisImu),
    turretMCB(turretMCB),
    kf(KF_A, KF_C, KF_Q, KF_R, KF_P0)
    {  
        // Set up the mecanum drive velocity calculation matrix
        wheelVelToChassisVelMat[X][LF] = 1;
        wheelVelToChassisVelMat[X][RF] = -1;
        wheelVelToChassisVelMat[X][LB] = 1;
        wheelVelToChassisVelMat[X][RB] = -1;
        wheelVelToChassisVelMat[Y][LF] = -1;
        wheelVelToChassisVelMat[Y][RF] = -1;
        wheelVelToChassisVelMat[Y][LB] = 1;
        wheelVelToChassisVelMat[Y][RB] = 1;
        wheelVelToChassisVelMat[R][LF] = -1.0 / chassis::WHEELBASE_HYPOTENUSE;
        wheelVelToChassisVelMat[R][RF] = -1.0 / chassis::WHEELBASE_HYPOTENUSE;
        wheelVelToChassisVelMat[R][LB] = -1.0 / chassis::WHEELBASE_HYPOTENUSE;
        wheelVelToChassisVelMat[R][RB] = -1.0 / chassis::WHEELBASE_HYPOTENUSE;
        wheelVelToChassisVelMat *= (chassis::WHEEL_RADIUS / 4);
    }

    void StandardTransformer::update() {
        // update odometry so it can be used to update
        updateOdometry();
        // update transforms with odometry data
        updateTransforms();
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

    const Transform<WorldFrame, ChassisFrame>& StandardTransformer::getWorldToChassisTransform() {
        worldToChassisTransform = compose<WorldFrame, ChassisIMUFrame, ChassisFrame>
            (worldToChassisIMUTransform, chassisIMUToChassisTransform);
        return worldToChassisTransform;
    }

    const Transform<WorldFrame, TurretIMUFrame>& StandardTransformer::getWorldToTurretIMUTransform() {
        worldToChassisTransform = compose<WorldFrame, ChassisIMUFrame, ChassisFrame>
            (worldToChassisIMUTransform, chassisIMUToChassisTransform);
        worldToTurretIMUTransform = compose<WorldFrame, ChassisFrame, TurretIMUFrame>
            (worldToChassisTransform, chassisToTurretIMUTransform);
        return worldToTurretIMUTransform;
    }

    const Transform<ChassisFrame, TurretIMUFrame>& StandardTransformer::getChassisToTurretIMUTransform() {
        return chassisToTurretIMUTransform;
    }

    const Transform<ChassisFrame, WorldFrame>& StandardTransformer::getChassisToWorldTransform() {
        worldToChassisTransform = compose<WorldFrame, ChassisIMUFrame, ChassisFrame>
            (worldToChassisIMUTransform, chassisIMUToChassisTransform);
        chassisToWorldTransform = worldToChassisTransform.getInverse();
        return chassisToWorldTransform;
    }

    const Transform<TurretIMUFrame, ChassisFrame>& StandardTransformer::getTurretIMUToChassisTransform() {
        turretIMUToChassisTransform = chassisToTurretIMUTransform.getInverse();
        return turretIMUToChassisTransform;
    }

    const Transform<CameraFrame, TurretIMUFrame>& StandardTransformer::getCameraToTurretIMUTransform() {
        cameraToTurretIMUTransform = TurretIMUToCameraTransform.getInverse();
        return cameraToTurretIMUTransform;
    }

    void StandardTransformer::updateTransforms() {
        // update all transforms that can't be derived from others
        worldToChassisIMUTransform = Transform<WorldFrame, ChassisIMUFrame>
            (-chassisWorldPosition.getX(), -chassisWorldPosition.getY(), -chassisWorldPosition.getZ(),
             -chassisWorldOrientation.getX(), -chassisWorldOrientation.getY(),-chassisWorldOrientation.getZ());
        
        TurretIMUToCameraTransform = Transform<TurretIMUFrame, CameraFrame>
            (0., TURRETIMU_TO_CAMERA_Y_OFFSET, 0., 
             turretWorldOrientation.getX(), turretWorldOrientation.getY(), turretWorldOrientation.getZ());

        turretIMUToGunTransform = Transform<TurretIMUFrame, GunFrame>
            (0.                          , TURRETIMU_TO_GUN_Y_OFFSET    , TURRETIMU_TO_GUN_Z_OFFSET,
             turretWorldOrientation.getX(), turretWorldOrientation.getY(), turretWorldOrientation.getZ());
    }

    void StandardTransformer::updateOdometry() {
    
        modm::Matrix<float, 3, 1> chassisVelocity = getActualVelocityChassisRelative();
        rotateChassisVectorToWorld(chassisVelocity);

        float accData[3] = {chassisImu.getAx(), chassisImu.getAx(), chassisImu.getAx()};
        modm::Matrix<float, 3, 1> chassisAcceleration = modm::Matrix<float, 3, 1>(accData);
        rotateChassisVectorToWorld(chassisAcceleration);

        // kf inputs are in the perspective of the world frame
        float nextKFInput[int(OdomInput::NUM_INPUTS)] = {};
        
        nextKFInput[int(OdomInput::VEL_X)] = chassisVelocity[0][0];
        nextKFInput[int(OdomInput::VEL_Y)] = chassisVelocity[1][0];
        nextKFInput[int(OdomInput::VEL_Z)] = chassisVelocity[2][0];

        nextKFInput[int(OdomInput::ACC_X)] = chassisAcceleration[0][0];
        nextKFInput[int(OdomInput::ACC_Y)] = chassisAcceleration[1][0];
        nextKFInput[int(OdomInput::ACC_Z)] = chassisAcceleration[2][0];

        // update kf, a new state matrix will be available following update
        kf.performUpdate(nextKFInput);

        // update chassis stored values (for use in location and rotation accessors)
        updateInternalOdomFromKF();
    }

    void StandardTransformer::updateInternalOdomFromKF() {
        const auto& state = kf.getStateVectorAsMatrix();

        // update the store odometry for easy access internally
        chassisWorldPosition.setX(state[int(OdomState::POS_X)]);
        chassisWorldPosition.setY(state[int(OdomState::POS_Y)]);
        chassisWorldPosition.setZ(state[int(OdomState::POS_Z)]);

        chassisWorldOrientation.setX(chassisImu.getRoll());
        chassisWorldOrientation.setY(chassisImu.getPitch());
        chassisWorldOrientation.setZ(chassisImu.getYaw());

        // we cannot query turret roll  
        turretWorldOrientation.setX(0);
        turretWorldOrientation.setY(turretMCB.getPitch());
        turretWorldOrientation.setZ(turretMCB.getYaw());
    }

    modm::Matrix<float, 3, 1> StandardTransformer::getActualVelocityChassisRelative() {
        modm::Matrix<float, WheelRPMIndex::NUM_MOTORS, 1> wheelVelocity;

        // init hasn't been called
        if (leftBackMotor == nullptr) return modm::Matrix<float, 3, 1>().zeroMatrix();

        if (!leftBackMotor->isMotorOnline() || !rightBackMotor->isMotorOnline()
        || !leftFrontMotor->isMotorOnline() || !rightFrontMotor->isMotorOnline())
            return modm::Matrix<float, 3, 1>().zeroMatrix();

        wheelVelocity[LF][0] = leftFrontMotor->getShaftRPM();
        wheelVelocity[RF][0] = rightFrontMotor->getShaftRPM();
        wheelVelocity[LB][0] = leftBackMotor->getShaftRPM();
        wheelVelocity[RB][0] = rightBackMotor->getShaftRPM();

        return wheelVelToChassisVelMat * convertRawRPM(wheelVelocity);
    }

      void StandardTransformer::rotateChassisVectorToWorld(modm::Matrix<float, 3, 1>& chassisRelVector) {
        // use a transform :P

        // our chassisToWorldTransform is 1 cycle out of date, so in the future we 
        // may want to extrapolate values to construct the up-to-date transform
        // but for now I'm lazy...

        // need to get a CMSISMat from chassisRelVector since Transforms take 
        // CMSISMats :(

        float data[3] = {chassisRelVector[0][0], chassisRelVector[1][0], chassisRelVector[2][0]};
        CMSISMat<3, 1> cmsisChassisRelVector = CMSISMat<3, 1>(data);

        // Transform.applyToVector only rotates, doesn't translate
        CMSISMat<3, 1> cmsisWorldRelVector = worldToChassisIMUTransform.getInverse()
                                                .applyToVector(cmsisChassisRelVector);

        // copy transformed positions back to original vector
        chassisRelVector[0][0] = cmsisWorldRelVector.data[0];
        chassisRelVector[1][0] = cmsisWorldRelVector.data[1];
        chassisRelVector[2][0] = cmsisWorldRelVector.data[2];
      }
}
