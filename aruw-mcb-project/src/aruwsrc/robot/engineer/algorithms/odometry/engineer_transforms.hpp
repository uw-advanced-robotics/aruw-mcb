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

namespace aruwsrc::engineer::algorithms::odometry
{
class EngineerTransforms
{
    using Transform = tap::algorithms::transforms::Transform;
    using Position = tap::algorithms::transforms::Position;
    using Orientation = tap::algorithms::transforms::Orientation;
    friend class EngineerTransformAdapter;

public:
    EngineerTransforms(const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry);

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

    inline const Transform& getChassisToArducam() const { return CHASSIS_TO_ARDUCAM; }

    inline const Transform& getWorldToArducam() const { return worldToArducam; }

protected:
    inline const tap::algorithms::odometry::Odometry2DInterface& getChassisOdometry() const
    {
        return chassisOdometry;
    }

private:
    const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry;

    // Transforms
    Transform worldToChassis;
    Transform worldToArducam;

    // Arducam offset
    const Transform CHASSIS_TO_ARDUCAM = Transform(Position(0, 0, 0), Orientation(0, 0, 0));
};

}  // namespace aruwsrc::engineer::algorithms::odometry

#endif  // ENGINEER_TRANSFORMS_HPP_
