/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef STANDARD_AND_HERO_TRANSFORMER_HPP_
#define STANDARD_AND_HERO_TRANSFORMER_HPP_

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/algorithms/transforms/transform.hpp"

#include "aruwsrc/control/turret/robot_turret_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/control/turret/yaw_turret_subsystem.hpp"

namespace aruwsrc::algorithms::transforms
{
/**
 * @brief transform provider for both the standard and hero
 */
class StandardAndHeroTransformer
{
public:
    StandardAndHeroTransformer(
        const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry,
        const aruwsrc::control::turret::RobotTurretSubsystem& turret);

    /**
     * @brief updates the transforms stored by the transformer
     * Should be called once per loop
     */
    void updateTransforms();

    inline const tap::algorithms::transforms::Transform& getWorldToChassis()
    {
        return worldToChassis;
    }

    inline const tap::algorithms::transforms::Transform& getWorldToTurret()
    {
        return worldToTurret;
    }

    inline const tap::algorithms::transforms::Transform& getChassisToTurret()
    {
        return chassisToTurret;
    }

private:
    const tap::algorithms::odometry::Odometry2DInterface& chassisOdometry;
    const aruwsrc::control::turret::RobotTurretSubsystem& turret;

    tap::algorithms::transforms::Transform worldToChassis;
    tap::algorithms::transforms::Transform worldToTurret;
    tap::algorithms::transforms::Transform chassisToTurret;
};

}  // namespace aruwsrc::algorithms::transforms

#endif  // STANDARD_AND_HERO_TRANSFORMER_HPP_
