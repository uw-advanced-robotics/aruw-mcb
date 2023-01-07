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

#ifndef STANDARD_TRANFORMER_HPP_
#define STANDARD_TRANFORMER_HPP_
// NOTE: this implementation wraps kfodometry to try to mimic the behavior
// 

// libraries needed by transforms
#include "tap/algorithms/transforms/transformer.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "aruwsrc/algorithms/transforms/standard_frames.hpp"

// for odometry
#include "aruwsrc/algorithms/odometry/chassis_kf_odometry.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/algorithms/odometry/chassis_world_yaw_observer_interface.hpp"


using namespace tap::algorithms;
using namespace tap::algorithms::transforms;

namespace aruwsrc::algorithms

{
    class StandardTransformer : public Transformer
    {
     public:
        /**
         * A transform provider that provides transforms for the standard
         * robot. 
         * @param chassisOdometry odometry used to update transforms
        */
        StandardTransformer
        (aruwsrc::algorithms::odometry::ChassisKFOdometry& chassisOdometry);
        
        /**
         * Update each transform with most recent encoder and IMU odometry data. This method
         * should be called every refresh.
        */
        void update();

        /**
         * Get World to Chassis transform
         * 
         * @returns World to Chassis transform
        */
        Transform<WorldFrame, ChassisFrame> StandardTransformer::getWorldToChassisTranform();

        /**
         * Get World to Turret transform
         * 
         * @returns World to Turret transform
        */
        Transform<WorldFrame, TurretFrame> StandardTransformer::getWorldToTurretTranform();

        /**
         * Get Chassis to Turret transform
         * 
         * @returns Chassis to Turret transform
        */
        Transform<ChassisFrame, TurretFrame> StandardTransformer::getChassisToTurretTranform();

        /**
         * Get Chassis to World transform
         * 
         * @returns Chassis to World transform
        */
        Transform<ChassisFrame, WorldFrame> StandardTransformer::getChassisToWorldTranform();
        
        /**
         * Get Turret to Chassis transform
         * 
         * @returns Turret to Chassis transform
        */
        Transform<TurretFrame, ChassisFrame> StandardTransformer::getTurretToChassisTranform();
     private:
        aruwsrc::algorithms::odometry::ChassisKFOdometry& privateOdom;
        /**
         * Updates the stored transforms for this cycle
        */
        void updateTransforms();

        Transform<WorldFrame, ChassisFrame> worldToChassisTransform;
        Transform<WorldFrame, TurretFrame> worldToTurretTransform;
        Transform<ChassisFrame, TurretFrame> chassisToTurretTransform;
        Transform<ChassisFrame, WorldFrame> chassisToWorldTransform;
        Transform<TurretFrame, ChassisFrame> turretToChassisTransform;

        float PLACEHOLDER_VAL = 0.0f;

    };
}


#endif // STANDARD_TRANFORMER_HPP_






// #include "tap/motor/dji_motor.hpp"
// #include "tap/communication/sensors/imu/imu_interface.hpp" // change this probably
// #include "tap/algorithms/transforms/transformer.hpp"
// #include "tap/algorithms/transforms/transform.hpp"
// #include "aruwsrc/algorithms/transforms/frames.hpp"
// #include "aruwsrc/algorithms/odometry/chassis_kf_odometry.hpp"

// using namespace tap::algorithms;

// namespace aruwsrc::algorithms
// {

// /**
//  * Standard-specific Transformer to handle and update world, chassis, 
//  * and turret frame transforms.
//  * 
//  * @param leftFrontMotor Left front motor
//  * @param leftBackMotor Left back motor
//  * @param rightFrontMotor Right front motor
//  * @param rightBackMotor Right back motor
//  * @param turretPitchMotor Turret pitch motor
//  * @param turretYawMotor Turret yaw motor
//  * @param chassisImu Chassis IMU
//  * @param turretImu Turret IMU
// */
// class StandardTransformer : public Transformer
// {
// public:
//     StandardTransformer::StandardTransformer (
//         tap::motor::DjiMotor& leftFrontMotor,
//         tap::motor::DjiMotor& leftBackMotor,
//         tap::motor::DjiMotor& rightFrontMotor,
//         tap::motor::DjiMotor& rightBackMotor,
//         tap::motor::DjiMotor& turretPitchMotor,
//         tap::motor::DjiMotor& turretYawMotor,
//         tap::communication::sensors::imu::ImuInterface& chassisImu,
//         tap::communication::sensors::imu::ImuInterface& turretImu
//     );
    
//     /**
//      * Get World to Chassis transform
//      * 
//      * @returns World to Chassis transform
//     */
//     Transform<WorldFrame, ChassisFrame> StandardTransformer::getWorldToChassisTranform();

//     /**
//      * Get World to Turret transform
//      * 
//      * @returns World to Turret transform
//     */
//     Transform<WorldFrame, TurretFrame> StandardTransformer::getWorldToTurretTranform();

//     /**
//      * Get Chassis to Turret transform
//      * 
//      * @returns Chassis to Turret transform
//     */
//     Transform<ChassisFrame, TurretFrame> StandardTransformer::getChassisToTurretTranform();

//     /**
//      * Get Chassis to World transform
//      * 
//      * @returns Chassis to World transform
//     */
//     Transform<ChassisFrame, WorldFrame> StandardTransformer::getChassisToWorldTranform();
    
//     /**
//      * Get Turret to Chassis transform
//      * 
//      * @returns Turret to Chassis transform
//     */
//     Transform<TurretFrame, ChassisFrame> StandardTransformer::getTurretToChassisTranform();

//     /**
//      * Update each transform with most recent encoder and IMU odometry data. This method
//      * should be called every refresh.
//     */
//     void StandardTransformer::update();

// private:
//     // tap::motor::DjiMotor& leftFrontMotor;
//     // tap::motor::DjiMotor& leftBackMotor;
//     // tap::motor::DjiMotor& rightFrontMotor;
//     // tap::motor::DjiMotor& rightBackMotor;
//     // tap::motor::DjiMotor& turretPitchMotor;
//     // tap::motor::DjiMotor& turretYawMotor;
//     // tap::communication::sensors::imu::ImuInterface& chassisImu;
//     // tap::communication::sensors::imu::ImuInterface& turretImu;

    // Transform<WorldFrame, ChassisFrame> worldToChassisTransform;
    // Transform<WorldFrame, TurretFrame> worldToTurretTransform;
    // Transform<ChassisFrame, TurretFrame> chassisToTurretTransform;
    // Transform<ChassisFrame, WorldFrame> chassisToWorldTransform;
    // Transform<TurretFrame, ChassisFrame> turretToChassisTransform;


//     // EVENTUALLY:
//     // have some internal kalman filter that keeps track of:
//     //  chassis (xyzabc)    (where abc = rotation in 3d)
//     //  turret  (xyzabc)

//     // FOR NOW:
//     //  just 2d stuff, wrap around a kfodometry for testing purposes
//     //  write tests for this
//     //  Then, move to our own implementation of the same behavior
//     //  see if it passes the same tests
//     //  then move to 3d
//     aruwsrc::algorithms::odometry::ChassisKFOdometry internalOdom;






















// // enum class OdomStateCartesian 
// //     {
// //         POS_X = 0,
// //         VEL_X,
// //         ACC_X,
// //         POS_Y,
// //         VEL_Y,
// //         ACC_Y,
// //         POS_Z,
// //         VEL_Z,
// //         ACC_Z,
// //         NUM_STATES,
// //     };

// //     enum class OdomInputCartesian
// //     {
// //         VEL_X = 0,
// //         ACC_X,
// //         VEL_Y,
// //         ACC_Y,
// //         VEL_Z,
// //         ACC_Z,
// //         NUM_INPUTS
// //     };

// //     enum class OdomStateRotation 
// //     {
// //         THETA_X = 0,
// //         OMEGA_X,
// //         ALPHA_X,
// //         THETA_Y,
// //         OMEGA_Y,
// //         ALPHA_Y,
// //         THETA_Z,
// //         OMEGA_Z,
// //         ALPHA_Z,
// //         NUM_STATES
// //     };

// //     enum class OdomInputRotation
// //     {
// //         THETA_X = 0,
// //         OMEGA_X,
// //         THETA_Y,
// //         OMEGA_Y,
// //         THETA_Z,
// //         OMEGA_Z,
// //         NUM_INPUTS
// //     };

// //     static constexpr int STATES_SQUARED =
// //         static_cast<int>(OdomStateCartesian::NUM_STATES) * static_cast<int>(OdomStateCartesian::NUM_STATES);
// //     static constexpr int INPUTS_SQUARED =
// //         static_cast<int>(OdomInputCartesian::NUM_INPUTS) * static_cast<int>(OdomInputCartesian::NUM_INPUTS);
// //     static constexpr int INPUTS_MULT_STATES =
// //         static_cast<int>(OdomInputCartesian::NUM_INPUTS) * static_cast<int>(OdomStateCartesian::NUM_STATES);

// //     /// Assumed time difference between calls to `update`, in seconds
// //     static constexpr float DT = 0.002f;

// //     // clang-format off
// //     static constexpr float KF_A[STATES_SQUARED] = {
// //         1, DT, 0.5 * DT * DT, 0, 0 , 0            ,
// //         0, 1 , DT           , 0, 0 , 0            ,
// //         0, 0 , 1            , 0, 0 , 0            ,
// //         0, 0 , 0            , 1, DT, 0.5 * DT * DT,
// //         0, 0 , 0            , 0, 1 , DT           ,
// //         0, 0 , 0            , 0, 0 , 1            ,
// //     };


// };

// }
