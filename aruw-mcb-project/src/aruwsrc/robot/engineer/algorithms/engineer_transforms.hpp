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
#ifndef ENGINEER_TRANSFORMS_HPP_
#define ENGINEER_TRANSFORMS_HPP_

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/algorithms/transforms/transform.hpp"

#include "modm/math/geometry/location_2d.hpp"

#include "engineer_kinematic_constants.hpp"
#include "point_mass.hpp"

namespace tap::communication::sensors::imu
{
class AbstractIMU;
}  // namespace tap::communication::sensors::imu
namespace aruwsrc::control::turret
{
class TurretSubsystem;
}
namespace aruwsrc::control::joint
{
class JointSubsystem;
}
namespace aruwsrc::engineer::wrist
{
class WristSubsystem;
}
namespace aruwsrc::communication::serial
{
class EngineerCVCommunication;
}

namespace aruwsrc::engineer::algorithms
{
/**
 * @note Frames are located immediately after the joint they're named after. Ex: Wrist
 * frame is located at the differential intersection, oriented towards the next joint.
 */
class EngineerTransforms
{
    using Transform = tap::algorithms::transforms::Transform;
    using Position = tap::algorithms::transforms::Position;
    using Vector = tap::algorithms::transforms::Vector;
    using Orientation = tap::algorithms::transforms::Orientation;
    friend class EngineerTransformAdapter;

public:
    EngineerTransforms(
        const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry,
        const tap::communication::sensors::imu::AbstractIMU& chassisImu,
        const aruwsrc::control::turret::TurretSubsystem& turret,
        const tap::communication::sensors::imu::AbstractIMU& turretPitchImu,
        const aruwsrc::control::joint::JointSubsystem& extension,
        const aruwsrc::engineer::wrist::WristSubsystem& wrist,
        const aruwsrc::control::joint::JointSubsystem& cubeStorage,
        aruwsrc::communication::serial::EngineerCVCommunication& engineerCVCommunication);

    void updateTransforms();

    inline void initialize() {}

    inline const Transform& getWorldToChassis() const { return worldToChassis; };

    inline uint32_t getLastComputedOdometryTime() const
    {
        return chassisOdometry.getLastComputedOdometryTime();
    }

    inline modm::Vector2f getChassisVelocity2d() const
    {
        return chassisOdometry.getCurrentVelocity2D();
    }

    inline const Transform& getWorldToTurretPitch() const { return worldToTurretPitch; }
    inline const Transform& getWorldToRealsense() const { return worldToRealsense; }
    inline const Transform& getWorldToReceptacle() const { return worldToReceptacle; }

    /**
     * Received-timestamp (ms) of the CV packet that produced the current worldToReceptacle, or
     * 0 if none has ever been received. This single field doubles as the validity flag
     * (>= 0 means valid) and the pose-age source: it is copied straight from
     * EngineerCVCommunication::getLastReceivedTimeMs() whenever worldToReceptacle is rebuilt.
     */
    inline int64_t getWorldToReceptacleReceivedTimeMs() const
    {
        return worldToReceptacleReceivedTimeMs;
    }
    inline bool isWorldToReceptacleValid() const { return worldToReceptacleReceivedTimeMs != -1; }
    inline const Transform& getCubeStore1ToEndEffector() const { return cubeStore1ToEndEffector; }
    inline const Transform& getCubeStore2ToEndEffector() const { return cubeStore2ToEndEffector; }
    inline const Transform& getEndEffectorToCubeDist() const { return endEffectorToCubeDist; }
    inline const Transform& getVtmGimbalToEndEffector() const { return vtmGimbalToEndEffector; }

    inline const PointMass& getCOMBeyondTurretPitch() const { return COMBeyondTurretPitch; }
    inline const PointMass& getCOMBeyondWrist() const { return COMBeyondWrist; }

    static Transform getHypotheticalChassisToTurretYaw(float yawAngle)
    {
        return Transform(CHASSIS_TO_TURRET_YAW_POS, Orientation(0, 0, yawAngle));
    }

    static Transform getHypotheticalTurretYawToTurretPitch(float pitchAngle)
    {
        return Transform(TURRET_YAW_TO_TURRET_PITCH_POS, Orientation(0, pitchAngle, 0));
    }

    static Transform getHypotheticalTurretPitchToExtension(float extension)
    {
        return Transform(
            TURRET_PITCH_TO_EXTENSION_ZERO_POS + Vector(extension, 0, 0),
            Orientation(0, 0, 0));
    }

protected:
    inline const tap::algorithms::odometry::Odometry2DInterface& getChassisOdometry() const
    {
        return chassisOdometry;
    }

private:
    const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry;
    const tap::communication::sensors::imu::AbstractIMU& chassisImu;
    const aruwsrc::control::turret::TurretSubsystem& turret;
    const tap::communication::sensors::imu::AbstractIMU& turretPitchImu;
    const aruwsrc::control::joint::JointSubsystem& extension;
    const aruwsrc::engineer::wrist::WristSubsystem& wrist;
    const aruwsrc::control::joint::JointSubsystem& cubeStorage;
    aruwsrc::communication::serial::EngineerCVCommunication& engineerCVCommunication;

    // Joint Transforms
    Transform worldToChassis;
    Transform chassisToTurretYaw;
    Transform turretYawToTurretPitch;
    Transform turretPitchToExtension;
    Transform extensionToWrist;

    Transform cubeStoreFrameToCubeStoreCenter;

    // Compound/Requested Transforms
    Transform worldToTurretPitch;
    Transform worldToRealsense;
    Transform worldToReceptacle;
    Transform worldToEndEffector;       // purely for debug
    Transform cubeStore1ToEndEffector;  // TODO: should be cube not EE
    Transform cubeStore2ToEndEffector;  // TODO: should be cube not EE
    Transform vtmGimbalToEndEffector;   // TODO: should be cube not EE
    Transform endEffectorToCubeDist;

    // Subtree Center of Masses
    PointMass COMBeyondTurretPitch;
    PointMass COMBeyondWrist;
    int64_t worldToReceptacleReceivedTimeMs;  // -1 = never received
};

}  // namespace aruwsrc::engineer::algorithms

#endif  // ENGINEER_TRANSFORMS_HPP_
