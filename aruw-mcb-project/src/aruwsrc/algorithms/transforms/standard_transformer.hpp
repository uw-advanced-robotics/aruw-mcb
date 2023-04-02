/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef STANDARD_TRANSFORMER_HPP_
#define STANDARD_TRANSFORMER_HPP_

// libraries needed by transforms
#include "tap/algorithms/transforms/transform.hpp"
#include "tap/algorithms/transforms/transformer.hpp"

#include "aruwsrc/algorithms/transforms/standard_frames.hpp"
#include "modm/math/geometry/vector3.hpp"
#include "modm/math/matrix.hpp"

// for odometry
#include "tap/algorithms/kalman_filter.hpp"
#include "aruwsrc/control/chassis/mecanum_chassis_subsystem.hpp"
#include "aruwsrc/robot/standard/standard_turret_subsystem.hpp"
// #include "aruwsrc/control/chassis/constants/chassis_constants.hpp"


using namespace tap::algorithms;
using namespace tap::algorithms::transforms;

namespace aruwsrc::algorithms::transforms

{
class StandardTransformer : public Transformer
{
public:
    /**
     * A transform provider that provides transforms for the standard
     * robot.
     */
    StandardTransformer(tap::communication::sensors::imu::mpu6500::Mpu6500& chassisImu);

    /**
     * Update each transform with most recent encoder and IMU odometry data. This method
     * should be called every refresh.
     */
    void update();

    /**
     * used to get const references to the instantiated motors
     * must be called before the standard transformer can do anything.
     *
     */
    void init(  const chassis::MecanumChassisSubsystem* chassisSubsystem,
                const aruwsrc::control::turret::StandardTurretSubsystem* turretSubsystem);


    // store the default (non-zero) values in transforms
    void initializeStaticTransforms();

    // resets each transform to its starting state
    void resetTransforms();

    // resets just one transform to an identity transform
    template <class SOURCE, class TARGET>
        void setIdentityTransform(Transform<SOURCE, TARGET>& transform) {
        // Initialize CMSISMat input to the transform
        float emptyPositionData[3] = {0., 0., 0.};
        CMSISMat<3, 1> cmsisPos(emptyPositionData);

        transform.updatePosition(cmsisPos);
        transform.updateRotation(0., 0., 0.);
    }

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
#ifndef ENV_UNIT_TESTS
private:
#endif
    /**
     * Updates the stored transforms for this cycle
     */
    void updateTransforms();

    /**
     * Updates the internal odometry so that the values
     * responsible for transforms are available
     */
    void updateOdometry();

    /**
     * Updates the stored odometry values (not the same place as KF)
     * probably delete this later
     */
    void updateInternalOdomFromKF();

    // Transforms that are dynamically updated
    Transform<WorldFrame, ChassisIMUFrame> worldToChassisIMUTransform;
    Transform<TurretIMUFrame, CameraFrame> TurretIMUToCameraTransform;
    Transform<TurretIMUFrame, GunFrame> turretIMUToGunTransform;

    // Transforms that are compositions
    Transform<WorldFrame, TurretIMUFrame> worldToTurretIMUTransform;
    Transform<WorldFrame, ChassisFrame> worldToChassisTransform;

    // Transforms that are inverses
    Transform<ChassisFrame, WorldFrame> chassisToWorldTransform;
    Transform<TurretIMUFrame, ChassisFrame> turretIMUToChassisTransform;
    Transform<CameraFrame, TurretIMUFrame> cameraToTurretIMUTransform;

    // Constant transforms
    Transform<ChassisFrame, TurretIMUFrame> chassisToTurretIMUTransform;
    Transform<ChassisIMUFrame, ChassisFrame> chassisIMUToChassisTransform;

    // References to all devices necessary for tracking odometry
    const chassis::MecanumChassisSubsystem* chassis  = nullptr;
    const control::turret::TurretSubsystem* turret = nullptr;
    tap::communication::sensors::imu::mpu6500::Mpu6500& chassisImu;

    /**
     * placeholder value used when constructing transforms before odometry data
     * is available
     * the use of this variable indicates the value in the transform is in an
     * uninitialized state
     */
    const float TRANSFORM_PLACEHOLDER_VAL = 0.0f;
    const float TURRETIMU_TO_CAMERA_Y_OFFSET = 0.09404;

    const float TURRETIMU_TO_GUN_Y_OFFSET = 0.01194;
    const float TURRETIMU_TO_GUN_Z_OFFSET = 0.04197;

    const float CHASSIS_TO_TURRET_Z_OFFSET = 0.40144;
    const float CHASSISIMU_TO_CHASSIS_X_OFFSET = 0.10568;
    const float CHASSISIMU_TO_CHASSIS_Z_OFFSET = 0.12172;

    // enums and matrix for calculating chassis velocity from raw motor RPM
    enum WheelRPMIndex
    {
        LF = 0,
        RF = 1,
        LB = 2,
        RB = 3,
        NUM_MOTORS,
    };

    enum ChassisVelIndex
    {
        X = 0,
        Y = 1,
        R = 2,
    };

    /**
     * Compute the acceleration of the chassis relative to itself
     * Returns a 3x1 matrix <ax, ay, az>
     */
    modm::Matrix<float, 3, 1> getAccelerationChassisRelative();

    /**
     * Fills nextKFInput with measurements taken from the chassis about the current
     * state of the robot.
     *
     * Specifically, fills in the chassis' velocity and acceleration in the
     * perspective of the world frame
     */
    void fillKFInput(float nextKFInput[]);

    // x,y,z location of chassis in world frame
    modm::Vector3f chassisWorldPosition;

    // rotation of chassis about x, y, z axes in world frame
    // (roll, pitch, yaw)
    modm::Vector3f chassisWorldOrientation;

    // rotation of chassis about x, y, z axes in world frame
    // (roll, pitch, yaw)
    modm::Vector3f turretWorldOrientation;

    // Kalman Filter enums
    enum class OdomInput
    {
        VEL_X = 0,
        ACC_X,
        VEL_Y,
        ACC_Y,
        VEL_Z,
        ACC_Z,
        POS_YAW,
        VEL_YAW,
        NUM_INPUTS,
    };

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
        POS_YAW,
        VEL_YAW,
        ACC_YAW,
        NUM_STATES,
    };

    /**
     * Rotates a chassis-relative vector <a,b,c> from the chassis frame
     * to the world frame
     * Performs the rotation in-place on the input variable (for some reason ... )
     */
    void rotateChassisVectorToWorld(modm::Matrix<float, 3, 1>& chassisRelVector);

    static constexpr int STATES_SQUARED =
        static_cast<int>(OdomState::NUM_STATES) * static_cast<int>(OdomState::NUM_STATES);
    static constexpr int INPUTS_SQUARED =
        static_cast<int>(OdomInput::NUM_INPUTS) * static_cast<int>(OdomInput::NUM_INPUTS);
static constexpr int INPUTS_MULT_STATES =
        static_cast<int>(OdomInput::NUM_INPUTS) * static_cast<int>(OdomState::NUM_STATES);

    tap::algorithms::KalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)> kf;

    static constexpr float DT = 0.002f;

    // clang-format off
    static constexpr float KF_A[STATES_SQUARED] = {
        1, DT, 0.5 * DT * DT, 0, 0 , 0            , 0, 0 , 0            , 0, 0 , 0            ,
        0, 1 , DT           , 0, 0 , 0            , 0, 0 , 0            , 0, 0 , 0            ,
        0, 0 , 1            , 0, 0 , 0            , 0, 0 , 0            , 0, 0 , 0            ,
        0, 0 , 0            , 1, DT, 0.5 * DT * DT, 0, 0 , 0            , 0, 0 , 0            ,
        0, 0 , 0            , 0, 1 , DT           , 0, 0 , 0            , 0, 0 , 0            ,
        0, 0 , 0            , 0, 0 , 1            , 0, 0 , 0            , 0, 0 , 0            ,
        0, 0 , 0            , 0, 0 , 0            , 1, DT, 0.5 * DT * DT, 0, 0 , 0            ,
        0, 0 , 0            , 0, 0 , 0            , 0, 1 , DT           , 0, 0 , 0            ,
        0, 0 , 0            , 0, 0 , 0            , 0, 0 , 1            , 0, 0 , 0            ,
        0, 0 , 0            , 0, 0 , 0            , 0, 0 , 0            , 1, DT, 0.5 * DT * DT,
        0, 0 , 0            , 0, 0 , 0            , 0, 0 , 0            , 0, 1 , DT           ,
        0, 0 , 0            , 0, 0 , 0            , 0, 0 , 0            , 0, 0 , 1            ,
    };

    static constexpr float KF_C[INPUTS_MULT_STATES] = {
        0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    };

    static constexpr float KF_Q[STATES_SQUARED] = {
        1E1, 0  , 0   , 0  , 0  , 0   , 0  , 0  , 0   , 0  , 0  , 0   ,
        0  , 1E0, 0   , 0  , 0  , 0   , 0  , 0  , 0   , 0  , 0  , 0   ,
        0  , 0  , 1E-1, 0  , 0  , 0   , 0  , 0  , 0   , 0  , 0  , 0   ,
        0  , 0  , 0   , 1E1, 0  , 0   , 0  , 0  , 0   , 0  , 0  , 0   ,
        0  , 0  , 0   , 0  , 1E0, 0   , 0  , 0  , 0   , 0  , 0  , 0   ,
        0  , 0  , 0   , 0  , 0  , 1E-1, 0  , 0  , 0   , 0  , 0  , 0   ,
        0  , 0  , 0   , 0  , 0  , 0   , 1E1, 0  , 0   , 0  , 0  , 0   ,
        0  , 0  , 0   , 0  , 0  , 0   , 0  , 1E0, 0   , 0  , 0  , 0   ,
        0  , 0  , 0   , 0  , 0  , 0   , 0  , 0  , 1E-1, 0  , 0  , 0   ,
        0  , 0  , 0   , 0  , 0  , 0   , 0  , 0  , 0   , 1E1, 0  , 0   ,
        0  , 0  , 0   , 0  , 0  , 0   , 0  , 0  , 0   , 0  , 1E0, 0   ,
        0  , 0  , 0   , 0  , 0  , 0   , 0  , 0  , 0   , 0  , 0  , 1E-1,
    };

    static constexpr float KF_R[INPUTS_SQUARED] = {
        1.0, 0  , 0  , 0  , 0  , 0  , 0  , 0  ,
        0  , 1.2, 0  , 0  , 0  , 0  , 0  , 0  ,
        0  , 0  , 1.0, 0  , 0  , 0  , 0  , 0  ,
        0  , 0  , 0  , 1.2, 0  , 0  , 0  , 0  ,
        0  , 0  , 0  , 0  , 1.0, 0  , 0  , 0  ,
        0  , 0  , 0  , 0  , 0  , 1.2, 0  , 0  ,
        0  , 0  , 0  , 0  , 0  , 0  , 1.0, 0  ,
        0  , 0  , 0  , 0  , 0  , 0  , 0  , 1.2,
    };

    static constexpr float KF_P0[STATES_SQUARED] = {
        1E3, 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  ,
        0  , 1E3, 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  ,
        0  , 0  , 1E3, 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  ,
        0  , 0  , 0  , 1E3, 0  , 0  , 0  , 0  , 0  , 0  , 0  ,
        0  , 0  , 0  , 0  , 1E3, 0  , 0  , 0  , 0  , 0  , 0  ,
        0  , 0  , 0  , 0  , 0  , 1E3, 0  , 0  , 0  , 0  , 0  ,
        0  , 0  , 0  , 0  , 0  , 0  , 1E3, 0  , 0  , 0  , 0  ,
        0  , 0  , 0  , 0  , 0  , 0  , 0  , 1E3, 0  , 0  , 0  ,
        0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 1E3, 0  , 0  ,
        0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 1E3, 0  ,
        0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 1E3,
    };    
    // clang-format on
};
}  // namespace aruwsrc::algorithms::transforms

#endif  // STANDARD_TRANSFORMER_HPP_