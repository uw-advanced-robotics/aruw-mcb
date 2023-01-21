// // NOtes
// // maintain internal odometry stuff, pretty much copying chassis_kf_odometry
// // 




// /*
//  * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
//  *
//  * This file is part of aruw-mcb.
//  *
//  * aruw-mcb is free software: you can redistribute it and/or modify
//  * it under the terms of the GNU General Public License as published by
//  * the Free Software Foundation, either version 3 of the License, or
//  * (at your option) any later version.
//  *
//  * aruw-mcb is distributed in the hope that it will be useful,
//  * but WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  * GNU General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License
//  * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
//  */



#include "aruwsrc/algorithms/transforms/standard_transformer.hpp"
#include "tap/algorithms/transforms/transformer.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "aruwsrc/algorithms/transforms/standard_frames.hpp"

#include "tap/control/chassis/chassis_subsystem_interface.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"



// #include "aruwsrc/control/chassis/chassis_subsystem.hpp"

using namespace tap::algorithms;
using namespace tap::algorithms::transforms;



// TODO: take in pitch motor, yaw motor
// later: don't take in chassis subystem

namespace aruwsrc::algorithms {

    StandardTransformer::StandardTransformer
    (
        const tap::motor::DjiMotor& leftBackMotor,
        const tap::motor::DjiMotor& rightBackMotor,
        const tap::motor::DjiMotor& leftFrontMotor,
        const tap::motor::DjiMotor& rightFrontMotor,
        tap::communication::sensors::imu::ImuInterface& chassisImu,
        tap::communication::sensors::imu::ImuInterface& turretImu
    )
    :

    // Transforms that are dynamically updated
    // worldToChassisIMUTransform(Transform<WorldFrame, ChassisIMUFrame>(PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL)),
    // TurretIMUToCameraTransform(Transform<TurretIMUFrame,CameraFrame>(0., 94.04, 0., 0. ,0., 0.)),
    // turretIMUToGunTransform(Transform<TurretIMUFrame, GunFrame> (0., 11.94, 41.97, 0., 0., 0.)),

    // Transforms that are compositions
    // worldToTurretIMUTransform(Transform<WorldFrame, TurretIMUFrame>(PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL)),
    // worldToChassisTransform(Transform<WorldFrame, ChassisFrame>(PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL)),

    // Transforms that are inverses
    // chassisToWorldTransform(Transform<ChassisFrame, WorldFrame>(PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL)),
    // turretIMUToChassisTransform(Transform<TurretIMUFrame, ChassisFrame>(PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL)),
    // cameraToTurretIMUTransform(Transform<CameraFrame, TurretIMUFrame>(PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL)),

    // Constant transforms
    // chassisToTurretIMUTransform(Transform<ChassisFrame,TurretIMUFrame>(0., 0., 401.44, 0., 0., 0.)),
    // chassisIMUToChassisTransform(Transform<ChassisIMUFrame, ChassisFrame> (105.68, 0., 121.72, 0., 0., 0.)),

    leftBackMotor(leftBackMotor),
    rightBackMotor(rightBackMotor),
    leftFrontMotor(leftFrontMotor),
    rightFrontMotor(rightFrontMotor),
    chassisImu(chassisImu),
    turretImu(turretImu),
    kf(KF_A, KF_C, KF_Q, KF_R, KF_P0),
          chassisAccelerationToMeasurementCovarianceInterpolator(
          CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT,
          MODM_ARRAY_SIZE(CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT))
    {  
        wheelVelToChassisVelMat[X][LF] = 1;
        wheelVelToChassisVelMat[X][RF] = -1;
        wheelVelToChassisVelMat[X][LB] = 1;
        wheelVelToChassisVelMat[X][RB] = -1;
        wheelVelToChassisVelMat[Y][LF] = -1;
        wheelVelToChassisVelMat[Y][RF] = -1;
        wheelVelToChassisVelMat[Y][LB] = 1;
        wheelVelToChassisVelMat[Y][RB] = 1;
        update();
    }

    void StandardTransformer::update() {
        // update odometry so it can be used to update
        updateOdometry();
        // update transforms with odometry data
        updateTransforms();
    }

    // const Transform<WorldFrame, ChassisFrame>& StandardTransformer::getWorldToChassisTransform() {
    //     worldToChassisTransform = compose<WorldFrame, ChassisIMUFrame, ChassisFrame>
    //     (worldToChassisIMUTransform, chassisIMUToChassisTransform);
    //     return worldToChassisTransform;
    // }

    // const Transform<WorldFrame, TurretIMUFrame>& StandardTransformer::getWorldToTurretIMUTransform() {
    //     worldToChassisTransform = compose<WorldFrame, ChassisIMUFrame, ChassisFrame>
    //     (worldToChassisIMUTransform, chassisIMUToChassisTransform);
    //     worldToTurretIMUTransform = compose<WorldFrame, ChassisFrame, TurretIMUFrame>
    //     (worldToChassisTransform, chassisToTurretIMUTransform);
    //     return worldToTurretIMUTransform;
    // }

    // const Transform<ChassisFrame, TurretIMUFrame>& StandardTransformer::getChassisToTurretIMUTransform() {
    //     return chassisToTurretIMUTransform;
    // }

    // const Transform<ChassisFrame, WorldFrame>& StandardTransformer::getChassisToWorldTransform() {
    //     worldToChassisTransform = compose<WorldFrame, ChassisIMUFrame, ChassisFrame>
    //     (worldToChassisIMUTransform, chassisIMUToChassisTransform);
    //     chassisToWorldTransform = worldToChassisTransform.getInverse();
    //     return chassisToWorldTransform;
    // }

    // const Transform<TurretIMUFrame, ChassisFrame>& StandardTransformer::getTurretIMUToChassisTransform() {
    //     turretIMUToChassisTransform = chassisToTurretIMUTransform.getInverse();
    //     return turretIMUToChassisTransform;
    // }

    // const Transform<CameraFrame, TurretIMUFrame>& StandardTransformer::getCameraToTurretIMUTransform() {
    //     cameraToTurretIMUTransform = TurretIMUToCameraTransform.getInverse();
    //     return cameraToTurretIMUTransform;
    // }

    void StandardTransformer::updateTransforms() {

        // get values necessary for update (in perspective of world frame)
        // auto chassisWorldPos = getChassisPositionWorldRelative();
        // auto chassisWorldOrientation = getChassisPitchRollYawWorldRelative();
        
        // Since there isn't an easy way to do the rotation matrix computation,
        // let's just construct a new transform with the 6-float constructor
        // worldToChassisTransform = Transform<WorldFrame, ChassisFrame>
        //     (-pos2D.getX(), -pos2D.getY(), PLACEHOLDER_VAL,
        //       PLACEHOLDER_VAL, PLACEHOLDER_VAL, -chassisYaw);

        // worldToChassisIMUTransform = Transform<WorldFrame, ChassisIMUFrame>
        // (-chassisWorldPosition.getX(), -chassisWorldPosition.getY(), -chassisWorldPosition.getZ(),
        //  -chassisWorldOrientation.getX(), -chassisWorldOrientation.getY(),-chassisWorldOrientation.getZ());

        // TurretIMUToCameraTransform = Transform<TurretIMUFrame, CameraFrame>
        // (0., 94.04, 0., 
        // turretImu.getRoll(), turretImu.getPitch(), turretImu.getYaw());

        // turretIMUToGunTransform = Transform<TurretIMUFrame, GunFrame>
        // (0.                , 11.94               ,             41.97,
        // turretImu.getRoll(), turretImu.getPitch(), turretImu.getYaw());
    }

    void StandardTransformer::updateOdometry() {
      // @todo: look into get to get yaw more confidently later, I'm 
      // not sure this getYaw() gives world-relative yaw
      float chassisYaw = chassisImu.getYaw();
    
      // get chassis velocity
        modm::Matrix<float, NUM_MOTORS, 1> wheelVelocity;

        wheelVelocity[LF][0] = leftFrontMotor.getShaftRPM();
        wheelVelocity[RF][0] = rightFrontMotor.getShaftRPM();
        wheelVelocity[LB][0] = leftBackMotor.getShaftRPM();
        wheelVelocity[RB][0] = rightBackMotor.getShaftRPM();

      auto chassisVelocity = getActualVelocityChassisRelative();
      // transform chassis velocity to world frame
      tap::control::chassis::ChassisSubsystemInterface::getVelocityWorldRelative(
        chassisVelocity,
        chassisYaw);

      // upate covariance
      updateMeasurementCovariance(chassisVelocity);
      // construct odominput
      float y[int(OdomInput::NUM_INPUTS)] = {};
      y[int(OdomInput::VEL_X)] = chassisVelocity[0][0];
      y[int(OdomInput::VEL_Y)] = chassisVelocity[1][0];
      y[int(OdomInput::VEL_Z)] = chassisVelocity[2][0];

      y[int(OdomInput::ACC_X)] = chassisImu.getAx();
      y[int(OdomInput::ACC_Y)] = chassisImu.getAy();
      y[int(OdomInput::ACC_Z)] = chassisImu.getAz();

      // Rotate acceleration in MCB frame to the world frame
      // @todo: figure out what to do with z acceleration
      tap::algorithms::rotateVector(
        &y[int(OdomInput::ACC_X)],
        &y[int(OdomInput::ACC_Y)],
        serial::VisionCoprocessor::MCB_ROTATION_OFFSET + chassisYaw);

      // update kf, a new state matrix will be available following update
      kf.performUpdate(y);

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
      chassisWorldOrientation.setZ(chassisImu.getYaw()
        + serial::VisionCoprocessor::MCB_ROTATION_OFFSET);

      // update turret orientation
      turretWorldOrientation.setX(turretImu.getRoll());
      turretWorldOrientation.setY(turretImu.getPitch());
      turretWorldOrientation.setZ(turretImu.getYaw());
    }

    void StandardTransformer::updateMeasurementCovariance(const modm::Matrix<float, 3, 1>& chassisVelocity) {
        const uint32_t curTime = tap::arch::clock::getTimeMicroseconds();
        const uint32_t dt = curTime - prevTime;
        prevTime = curTime;

        if (prevTime == 0) 
        {
            return;
        }

        chassisMeasuredDeltaVelocity.x = tap::algorithms::lowPassFilter(
            chassisMeasuredDeltaVelocity.x,
            chassisVelocity[0][0] - prevChassisVelocity[0][0],
            CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA);

        chassisMeasuredDeltaVelocity.y = tap::algorithms::lowPassFilter(
            chassisMeasuredDeltaVelocity.y,
            chassisVelocity[1][0] - prevChassisVelocity[1][0],
            CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA);

        chassisMeasuredDeltaVelocity.z = tap::algorithms::lowPassFilter(
            chassisMeasuredDeltaVelocity.z,
            chassisVelocity[2][0] - prevChassisVelocity[2][0],
            CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA);

        const float accelMagnitude =
            chassisMeasuredDeltaVelocity.getLength() * 1E6 / static_cast<float>(dt);

        const float velocityCovariance =
            chassisAccelerationToMeasurementCovarianceInterpolator.interpolate(accelMagnitude);

        kf.getMeasurementCovariance()[0] = velocityCovariance;
        kf.getMeasurementCovariance()[2 * static_cast<int>(OdomInput::NUM_INPUTS) + 2] =
            velocityCovariance;

    }

    modm::Matrix<float, 3, 1> StandardTransformer::getActualVelocityChassisRelative() {
        modm::Matrix<float, WheelRPMIndex::NUM_MOTORS, 1> wheelVelocity;

        wheelVelocity[LF][0] = leftFrontMotor.getShaftRPM();
        wheelVelocity[RF][0] = rightFrontMotor.getShaftRPM();
        wheelVelocity[LB][0] = leftBackMotor.getShaftRPM();
        wheelVelocity[RB][0] = rightBackMotor.getShaftRPM();
        modm::Matrix<float, 2, 1> planarXYVelocity = wheelVelToChassisVelMat * convertRawRPM(wheelVelocity);
        modm::Matrix<float, 3, 1> chassisVelocity = modm::Matrix<float, 3, 1>();
        chassisVelocity[0][0] = planarXYVelocity[0][0];
        chassisVelocity[1][0] = planarXYVelocity[1][0];
        chassisVelocity[1][0] = 0;
        return chassisVelocity;
    }

      void StandardTransformer::getVelocityWorldRelative(
        modm::Matrix<float, 3, 1>& chassisRelativeVelocity) {

            // construct an updated transform using chassis roll, pitch, yaw 
            // to transform chassis velocity to world frame
            Transform<ChassisIMUFrame, WorldFrame> tempTransform = 
                Transform<ChassisIMUFrame, WorldFrame>
                (0.0, 0.0, 0.0, chassisImu.getRoll(), chassisImu.getPitch(), chassisImu.getYaw());


            // create temporary CMSISMats to get around parameter type mismatch
            float data[3] = {chassisRelativeVelocity[0][0], chassisRelativeVelocity[1][0], chassisRelativeVelocity[2][0]};
            CMSISMat<3, 1> temp = CMSISMat<3, 1>(data);

            // apply the transform
            CMSISMat<3, 1> chassisVWorldRelative = tempTransform.applyToVector(temp);


            // copy transformed positions back to original vector
            chassisRelativeVelocity[0][0] = chassisVWorldRelative.data[0];
            chassisRelativeVelocity[1][0] = chassisVWorldRelative.data[1];
            chassisRelativeVelocity[2][0] = chassisVWorldRelative.data[2];
            // Now xyz velocity is transformed to world frame
        }

}
