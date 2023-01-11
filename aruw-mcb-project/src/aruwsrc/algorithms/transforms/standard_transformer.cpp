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

namespace aruwsrc::algorithms {

    StandardTransformer::StandardTransformer
    (
      const tap::control::chassis::ChassisSubsystemInterface& chassisSubsystem,
      tap::communication::sensors::imu::ImuInterface& chassisImu,
      tap::communication::sensors::imu::ImuInterface& turretImu
    ) 
    :

    // ugly hack for since transform has no default constructor
    // but also allows us to set up any values in static transforms
    worldToChassisTransform(Transform<WorldFrame, ChassisFrame>(PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL)),
    worldToTurretTransform(Transform<WorldFrame, TurretFrame>(PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL)),
    chassisToTurretTransform(Transform<ChassisFrame, TurretFrame>(PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL)),
    chassisToWorldTransform(Transform<ChassisFrame, WorldFrame>(PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL)),
    turretToChassisTransform(Transform<TurretFrame, ChassisFrame>(PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL)),
    turretToWorldTransform(Transform<TurretFrame, WorldFrame>(PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL)),
    // chassisToTurretPivotTransform = static
    chassisToTurretPivotTransform(Transform<ChassisFrame, TurretPivotFrame>(PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL, PLACEHOLDER_VAL)),
    chassisSubsystem(chassisSubsystem),
    chassisImu(chassisImu),
    turretImu(turretImu),
    kf(KF_A, KF_C, KF_Q, KF_R, KF_P0)
    {  
        update();
    }

    void StandardTransformer::update() {
        // update odometry so it can be used to update
        updateOdometry();
        // update transforms with odometry data
        updateTransforms();
    }

    void StandardTransformer::updateTransforms() {

        // get values necessary for update (in perspective of world frame)
        // auto chassisWorldPos = getChassisPositionWorldRelative();
        // auto chassisWorldOrientation = getChassisPitchRollYawWorldRelative();
        
        // Since there isn't an easy way to do the rotation matrix computation,
        // let's just construct a new transform with the 6-float constructor
        // worldToChassisTransform = Transform<WorldFrame, ChassisFrame>
        //     (-pos2D.getX(), -pos2D.getY(), PLACEHOLDER_VAL,
        //       PLACEHOLDER_VAL, PLACEHOLDER_VAL, -chassisYaw);

        worldToChassisTransform = Transform<WorldFrame, ChassisFrame>
        (-chassisWorldPosition.getX(), -chassisWorldPosition.getY(), -chassisWorldPosition.getZ(),
         -chassisWorldOrientation.getX(), -chassisWorldOrientation.getY(),-chassisWorldOrientation.getZ());
        
        chassisToWorldTransform = worldToChassisTransform.getInverse(worldToChassisTransform);



        // chassisframe to turret pitch should be completely static
        
        // turretpitch to turret should have static position, dynamic
        // rotation coming from turret imu

        // transforms left to do

        // for turret transforms:
            // with current odometry, we can't do any turret transforms
            // chassis -> turret pivot can be static
            // turret pivot -> end of turret is dynamic only in rotation
            // chassis -> end of turret is just the composition of those
        
            // turret -> camera also static
            // chassis -> camera = chassis->turret * turret -> camera

            
    }

    void StandardTransformer::updateOdometry() {
      // @todo: look into get to get yaw more confidently later, I'm 
      // not sure this getYaw() gives world-relative yaw
      float chassisYaw = chassisImu.getYaw();
    
      // get chassis velocity
      auto chassisVelocity = chassisSubsystem.getActualVelocityChassisRelative();
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

      // Update chassis orientation
      // @note: rotation is not tracked by KF, so not 
      //  sure how orientation is handled
        //  ask Manoli how rotations are stored in imu: is the 
        //  is the 0,0,0 orientation the one that the 
        //  chassis starts with. If so, each imu has 
        //  a different world frame
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

    }
}
