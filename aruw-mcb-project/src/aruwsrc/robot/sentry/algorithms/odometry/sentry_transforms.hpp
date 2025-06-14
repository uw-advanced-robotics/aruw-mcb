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
#ifndef SENTRY_TRANSFORMS_HPP_
#define SENTRY_TRANSFORMS_HPP_

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/algorithms/transforms/transform.hpp"

#include "aruwsrc/control/turret/yaw_turret_subsystem.hpp"
#include "aruwsrc/robot/sentry/turret/sentry_turret_minor_subsystem.hpp"
#include "modm/math/geometry/location_2d.hpp"

namespace aruwsrc::sentry::algorithms::odometry
{
class SentryTransforms
{
    using Transform = tap::algorithms::transforms::Transform;
    using Position = tap::algorithms::transforms::Position;
    using Orientation = tap::algorithms::transforms::Orientation;
    friend class SentryTransformAdapter;

public:
    struct SentryTransformConfig
    {
        // Offset from turret minor yaw axis to turret major yaw axis (should only be in the
        // y-direction of the turret major frame)
        const float turretMinorOffset;
    };

    SentryTransforms(
        const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry,
        const aruwsrc::control::turret::YawTurretSubsystem& turretMajor,
        const aruwsrc::sentry::turret::SentryTurretMinorSubsystem& turretLeft,
        const aruwsrc::sentry::turret::SentryTurretMinorSubsystem& turretRight,
        const SentryTransformConfig& config);

    void updateTransforms();

    inline void initialize() {}

    inline const Transform& getWorldToChassis() const { return worldToChassis; };
    inline const Transform& getWorldToTurretMajor() const { return worldToTurretMajor; };
    inline const Transform& getWorldToTurretLeft() const { return worldToTurretLeft; };
    inline const Transform& getWorldToTurretRight() const { return worldToTurretRight; };

    inline const Transform& getChassisToMajor() const { return chassisToTurretMajor; };

    inline const Transform& getWorldToVTM() const { return worldToVTM; }

    // If you pass a wrong turretID, the right turret will automatically be returned.
    inline const Transform& getWorldToTurret(int turretID) const
    {
        if (turretID == turretLeft.getTurretID())
        {
            return worldToTurretLeft;
        }
        else
        {
            return worldToTurretRight;
        }
    }

    inline const Transform& getMajorToTurretLeft() const { return turretMajorToTurretLeft; };

    inline const Transform& getMajorToTurretRight() const { return turretMajorToTurretRight; };

    inline const Transform& getMajorToMinor(uint8_t turretId) const
    {
        if (turretId == turretLeft.getTurretID())
        {
            return turretMajorToTurretLeft;
        }
        else
        {
            return turretMajorToTurretRight;
        }
    };

    inline uint32_t getLastComputedOdometryTime() const
    {
        return chassisOdometry.getLastComputedOdometryTime();
    }

    inline modm::Vector2f getChassisVelocity2d() const
    {
        return chassisOdometry.getCurrentVelocity2D();
    }

    // If you pass a wrong cameraID, the first camera will automatically be returned.
    inline const Transform& getChassisToArducam(uint8_t cameraID) const
    {
        switch (cameraID)
        {
            case 0:
                return chassisToArducam0;
            case 1:
                return chassisToArducam1;
            case 2:
                return chassisToArducam2;
            case 3:
                return chassisToArducam3;
            default:
                return chassisToArducam0;
        }
    }

protected:
    inline const tap::algorithms::odometry::Odometry2DInterface& getChassisOdometry() const
    {
        return chassisOdometry;
    }

private:
    SentryTransformConfig config;

    const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry;
    const aruwsrc::control::turret::YawTurretSubsystem& turretMajor;
    const aruwsrc::sentry::turret::SentryTurretMinorSubsystem& turretLeft;
    const aruwsrc::sentry::turret::SentryTurretMinorSubsystem& turretRight;

    // Transforms
    Transform worldToChassis;
    Transform worldToTurretMajor;
    Transform worldToTurretLeft;
    Transform worldToTurretRight;
    Transform worldToVTM;
    Transform chassisToArducam0, chassisToArducam1, chassisToArducam2, chassisToArducam3;

    // Intermediary transforms
    Transform chassisToTurretMajor;
    Transform turretMajorToTurretLeft;
    Transform turretMajorToTurretRight;

    // Arducam offsets
    Transform ARDUCAM1_OFFSET = Transform(Position(-0.3, -0.18, 0), Orientation(0, 0, modm::toRadian(145))); // Back right
    Transform ARDUCAM2_OFFSET = Transform(Position(0.3, -0.18, 0), Orientation(0, 0, 0)); // Front right
    Transform ARDUCAM3_OFFSET = Transform(Position(-0.3, 0.18, 0), Orientation(0, 0, 0)); // Back left
    Transform ARDUCAM4_OFFSET = Transform(Position(0.3, 0.18, 0), Orientation(0, 0, 0)); // Front left
};

}  // namespace aruwsrc::sentry::algorithms::odometry

#endif  // SENTRY_TRANSFORMS_HPP_
