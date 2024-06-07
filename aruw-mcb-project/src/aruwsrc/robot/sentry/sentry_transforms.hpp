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
#include "modm/math/geometry/location_2d.hpp"

#include "sentry_minor_world_orientation_provider.hpp"
#include "sentry_turret_minor_subsystem.hpp"

namespace aruwsrc::sentry
{
class SentryTransforms
{
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
        const aruwsrc::control::sentry::SentryTurretMinorSubsystem& turretLeft,
        aruwsrc::control::turret::SentryMinorWorldOrientationProvider&
            turretLeftOrientationProvider,
        const aruwsrc::control::sentry::SentryTurretMinorSubsystem& turretRight,
        aruwsrc::control::turret::SentryMinorWorldOrientationProvider&
            turretRightOrientationProvider,
        const SentryTransformConfig& config);

    void updateTransforms();

    inline void initialize()
    {
        turretRightOrientationProvider.initialize(getWorldToTurretMajor());
        turretLeftOrientationProvider.initialize(getWorldToTurretMajor());
    }

    inline const tap::algorithms::transforms::Transform& getWorldToChassis() const
    {
        return worldToChassis;
    };
    inline const tap::algorithms::transforms::Transform& getWorldToTurretMajor() const
    {
        return worldToTurretMajor;
    };
    inline const tap::algorithms::transforms::Transform& getWorldToTurretLeft() const
    {
        return worldToTurretLeft;
    };
    inline const tap::algorithms::transforms::Transform& getWorldToTurretRight() const
    {
        return worldToTurretRight;
    };

    inline const tap::algorithms::transforms::Transform& getChassisToMajor() const
    {
        return chassisToTurretMajor;
    };

    // If you pass a wrong turretID, the right turret will automatically be returned.
    inline const tap::algorithms::transforms::Transform& getWorldToTurret(int turretID) const
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

    inline const tap::algorithms::transforms::Transform& getMajorToTurretLeft() const
    {
        return turretMajorToTurretLeft;
    };

    inline const tap::algorithms::transforms::Transform& getMajorToTurretRight() const
    {
        return turretMajorToTurretRight;
    };

    inline const tap::algorithms::transforms::Transform& getMajorToMinor(uint8_t turretId) const
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

protected:
    inline const tap::algorithms::odometry::Odometry2DInterface& getChassisOdometry() const
    {
        return chassisOdometry;
    }

private:
    SentryTransformConfig config;

    const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry;
    const aruwsrc::control::turret::YawTurretSubsystem& turretMajor;
    const aruwsrc::control::sentry::SentryTurretMinorSubsystem& turretLeft;
    aruwsrc::control::turret::SentryMinorWorldOrientationProvider& turretLeftOrientationProvider;
    const aruwsrc::control::sentry::SentryTurretMinorSubsystem& turretRight;
    aruwsrc::control::turret::SentryMinorWorldOrientationProvider& turretRightOrientationProvider;

    // Transforms
    tap::algorithms::transforms::Transform worldToChassis;
    tap::algorithms::transforms::Transform worldToTurretMajor;
    tap::algorithms::transforms::Transform worldToTurretLeft;
    tap::algorithms::transforms::Transform worldToTurretRight;

    // Intermediary transforms
    tap::algorithms::transforms::Transform chassisToTurretMajor;
    tap::algorithms::transforms::Transform turretMajorToTurretLeft;
    tap::algorithms::transforms::Transform turretMajorToTurretRight;
};

}  // namespace aruwsrc::sentry

#endif  // SENTRY_TRANSFORMS_HPP_
