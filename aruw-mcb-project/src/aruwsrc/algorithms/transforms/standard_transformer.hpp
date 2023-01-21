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
#include "tap/algorithms/kalman_filter.hpp"

#include "modm/math/geometry/vector3.hpp"
#include "modm/math/matrix.hpp"

// for odometry
#include "aruwsrc/algorithms/odometry/chassis_kf_odometry.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/algorithms/odometry/chassis_world_yaw_observer_interface.hpp"
// #include ""


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
        (      
        const tap::motor::DjiMotor& leftBackMotor,
        const tap::motor::DjiMotor& rightBackMotor,
        const tap::motor::DjiMotor& leftFrontMotor,
        const tap::motor::DjiMotor& rightFrontMotor,
        tap::communication::sensors::imu::ImuInterface& chassisImu,
        tap::communication::sensors::imu::ImuInterface& turretImu
        );

        /**
         * Update each transform with most recent encoder and IMU odometry data. This method
         * should be called every refresh.
        */
        void update();

        enum ChassisVelIndex
        {
            X = 0,
            Y = 1,
        };

        enum WheelRPMIndex
        {
            LF = 0,
            RF,
            LB,
            RB,
            NUM_MOTORS,
        };

        modm::Matrix<float, 2, 4> wheelVelToChassisVelMat;

        /**
         * Get World to Chassis transform
         * 
         * @returns World to Chassis transform
        */
        const Transform<WorldFrame, ChassisFrame>& getWorldToChassisTransform();

        /**
         * Get World to Turret transform
         * 
         * @returns World to Turret transform
        */
        const Transform<WorldFrame, TurretIMUFrame>& getWorldToTurretIMUTransform();

        /**
         * Get Chassis to Turret transform
         * 
         * @returns Chassis to Turret transform
        */
        const Transform<ChassisFrame, TurretIMUFrame>& getChassisToTurretIMUTransform();

        /**
         * Get Chassis to World transform
         * 
         * @returns Chassis to World transform
        */
        const Transform<ChassisFrame, WorldFrame>& getChassisToWorldTransform();
        
        /**
         * Get Turret to Chassis transform
         * 
         * @returns Turret to Chassis transform
        */
        const Transform<TurretIMUFrame, ChassisFrame>& getTurretIMUToChassisTransform();

        /**
         * Get Camera to Turret transform
         * 
         * @returns Camera to Turret transform
        */
        const Transform<CameraFrame, TurretIMUFrame>& getCameraToTurretIMUTransform();
     private:
        float turretIMUToChassisYawOffset   = -1.0f;
        float turretIMUToChassisPitchOffset = -1.0f;
        float turretIMUToChassisRollOffset  = -1.0f;
        /**
         * Updates the stored transforms for this cycle
        */
        void updateTransforms();

        /**
         * Updates the internal odometry so that the values
         * responsible for transforms are available
        */
        void updateOdometry();

        float PLACEHOLDER_VAL = 0.0f;

        // Transforms that are dynamically updated
        // Transform<WorldFrame, ChassisIMUFrame> worldToChassisIMUTransform;
        // Transform<TurretIMUFrame, CameraFrame> TurretIMUToCameraTransform;
        // Transform<TurretIMUFrame, GunFrame> turretIMUToGunTransform;

        // Transforms that are compositions
        // Transform<WorldFrame, TurretIMUFrame> worldToTurretIMUTransform;
        // Transform<WorldFrame, ChassisFrame> worldToChassisTransform;

        // Transforms that are inverses
        // Transform<ChassisFrame, WorldFrame> chassisToWorldTransform;
        // Transform<TurretIMUFrame, ChassisFrame> turretIMUToChassisTransform;
        // Transform<CameraFrame, TurretIMUFrame> cameraToTurretIMUTransform;

        // Constant transforms
        // Transform<ChassisFrame, TurretIMUFrame> chassisToTurretIMUTransform;
        // Transform<ChassisIMUFrame, ChassisFrame> chassisIMUToChassisTransform;

        // References to all devices necessary for tracking odometry
        const tap::motor::DjiMotor& leftBackMotor;
        const tap::motor::DjiMotor& rightBackMotor;
        const tap::motor::DjiMotor& leftFrontMotor;
        const tap::motor::DjiMotor& rightFrontMotor;
        tap::communication::sensors::imu::ImuInterface& chassisImu;
        tap::communication::sensors::imu::ImuInterface& turretImu;

        // kalman filter stuff for keeping track of chassis position
        enum class OdomState
        {
            POS_X = 0,
            VEL_X,
            ACC_X,
            POS_Y,
            VEL_Y,
            ACC_Y,
            POS_Z,
            VEL_Z,
            ACC_Z,
            NUM_STATES,
        };

        enum class OdomInput
        {
            VEL_X = 0,
            ACC_X,
            VEL_Y,
            ACC_Y,
            VEL_Z,
            ACC_Z,
            NUM_INPUTS,
        };


        static constexpr float CHASSIS_GEARBOX_RATIO = (187.0f / 3591.0f);

        static constexpr int STATES_SQUARED =
          static_cast<int>(OdomState::NUM_STATES) * static_cast<int>(OdomState::NUM_STATES);
        static constexpr int INPUTS_SQUARED =
          static_cast<int>(OdomInput::NUM_INPUTS) * static_cast<int>(OdomInput::NUM_INPUTS);
        static constexpr int INPUTS_MULT_STATES =
          static_cast<int>(OdomInput::NUM_INPUTS) * static_cast<int>(OdomState::NUM_STATES);
            
        /// Assumed time difference between calls to `update`, in seconds
        static constexpr float DT = 0.002f;
        
        // clang-format off
        static constexpr float KF_A[STATES_SQUARED] = {
          1, DT, 0.5 * DT * DT, 0, 0 , 0            , 0, 0, 0,
          0, 1 , DT           , 0, 0 , 0            , 0, 0, 0,
          0, 0 , 1            , 0, 0 , 0            , 0, 0, 0,
          0, 0 , 0            , 1, DT, 0.5 * DT * DT, 0, 0, 0,
          0, 0 , 0            , 0, 1 , DT           , 0, 0, 0,
          0, 0 , 0            , 0, 0 , 1            , 0, 0, 0,
          0, 0 , 0            , 0, 0 , 0            , 1, DT, 0.5 * DT * DT,
          0, 0 , 0            , 0, 0 , 0            , 0, 1 , DT,
          0, 0 , 0            , 0, 0 , 0            , 0, 0, 1,
        };

        static constexpr float KF_C[INPUTS_MULT_STATES] = {
          0, 1, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 1, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 1, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 1, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 1,
        };

        static constexpr float KF_Q[STATES_SQUARED] = {
          1E1, 0  , 0   , 0  , 0  , 0   , 0  , 0  , 0   ,
          0  , 1E0, 0   , 0  , 0  , 0   , 0  , 0  , 0   ,
          0  , 0  , 1E-1, 0  , 0  , 0   , 0  , 0  , 0   ,
          0  , 0  , 0   , 1E1, 0  , 0   , 0  , 0  , 0   ,
          0  , 0  , 0   , 0  , 1E0, 0   , 0  , 0  , 0   ,
          0  , 0  , 0   , 0  , 0  , 1E-1, 0  , 0  , 0   ,
          0  , 0  , 0   , 0  , 0  , 0   , 1E1, 0  , 0   ,
          0  , 0  , 0   , 0  , 0  , 0   , 0  , 1E0, 0   ,
          0  , 0  , 0   , 0  , 0  , 0   , 0  , 0  , 1E-1,
        };

        static constexpr float KF_R[INPUTS_SQUARED] = {
          1.0, 0  , 0  , 0  , 0  , 0  ,
          0  , 1.2, 0  , 0  , 0  , 0  ,
          0  , 0  , 1.0, 0  , 0  , 0  ,
          0  , 0  , 0  , 1.2, 0  , 0  ,
          0  , 0  , 0  , 0  , 1.0, 0  ,
          0  , 0  , 0  , 0  , 0  , 1.2,
        };

        static constexpr float KF_P0[STATES_SQUARED] = {
          1E3, 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  ,
          0  , 1E3, 0  , 0  , 0  , 0  , 0  , 0  , 0  ,
          0  , 0  , 1E3, 0  , 0  , 0  , 0  , 0  , 0  ,
          0  , 0  , 0  , 1E3, 0  , 0  , 0  , 0  , 0  ,
          0  , 0  , 0  , 0  , 1E3, 0  , 0  , 0  , 0  ,
          0  , 0  , 0  , 0  , 0  , 1E3, 0  , 0  , 0  ,
          0  , 0  , 0  , 0  , 0  , 0  , 1E3, 0  , 0  ,
          0  , 0  , 0  , 0  , 0  , 0  , 0  , 1E3, 0  ,
          0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 1E3,
        };
        // clang-format on

        // Kalman filter for keeping track of chassis (x, y, z)
        tap::algorithms::KalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)> kf;

        // x,y,z location of chassis in world frame
        modm::Vector3f chassisWorldPosition;

        // rotation of chassis about x, y, z axes in world frame
        // (roll, pitch, yaw)
        modm::Vector3f chassisWorldOrientation;

        // rotation of turret about x, y, z axes in world frame
        // (roll, pitch, yaw)
        modm::Vector3f turretWorldOrientation;

        void updateInternalOdomFromKF();

        void updateMeasurementCovariance(const modm::Matrix<float, 3, 1>& chassisVelocity);
        static constexpr float MAX_ACCELERATION = 8.0f;

        static constexpr modm::Pair<float, float> CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT[] =
        {
            {0, 1E0},
            {MAX_ACCELERATION, 1E2},
        };

        static constexpr float CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA = 0.01f;

        /// Chassis measured change in velocity since the last time `update` was called, in the chassis
        /// frame
        modm::Vector3f chassisMeasuredDeltaVelocity;

        modm::interpolation::Linear<modm::Pair<float, float>>
            chassisAccelerationToMeasurementCovarianceInterpolator;

        /// Previous time `update` was called, in microseconds
        uint32_t prevTime = 0;
        modm::Matrix<float, 3, 1> prevChassisVelocity;

        modm::Matrix<float, 3, 1> getActualVelocityChassisRelative();

        inline modm::Matrix<float, 4, 1> convertRawRPM(const modm::Matrix<float, 4, 1>& mat) const
        {
            static constexpr float ratio = 2.0f * M_PI * CHASSIS_GEARBOX_RATIO / 60.0f;
            return mat * ratio;
        }

        /**
         * Transforms the chassis relative velocity of the form <vx, vy, vz> 
         * into world relative frame, given some particular chassis
         * Transforms the input matrix chassisRelativeVelocity. Units: m/s
         */
        void getVelocityWorldRelative(modm::Matrix<float, 3, 1>& chassisRelativeVelocity);
    };
}


#endif // STANDARD_TRANFORMER_HPP_