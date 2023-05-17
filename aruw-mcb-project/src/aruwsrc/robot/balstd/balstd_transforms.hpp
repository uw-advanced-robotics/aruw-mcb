/*
 * Copyright (c) 2021-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BALSTD_TRANSFORMS_HPP_
#define BALSTD_TRANSFORMS_HPP_

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/algorithms/transforms/frame.hpp"
#include "tap/algorithms/transforms/transform.hpp"

#include "aruwsrc/control/chassis/balancing/balancing_chassis_subsystem.hpp"
#include "aruwsrc/robot/balstd/balstd_frames.hpp"
#include "aruwsrc/robot/standard/standard_turret_subsystem.hpp"

using namespace tap::algorithms::transforms;

namespace aruwsrc::balstd::transforms
{

class Transformer
{
public:
    Transformer(
        const aruwsrc::chassis::BalancingChassisSubsystem& chassis,
        const aruwsrc::control::turret::StandardTurretSubsystem& turret);

    Transform<World, Chassis> const& getWorldToChassis() const { return worldToChassis; }

    Transform<Chassis, Turret> const& getChassisToTurret() const { return chassisToTurret; }

    Transform<World, Turret> const& getWorldToTurret() const { return worldToTurret; }

    Transform<Turret, Chassis> const& getTurretToChassis() { return turretToChassis; }

    Transform<Chassis, World> const& getChassisToWorld() { return chassisToWorld; }

    void updateTransforms();

private:
    Transform<World, Chassis> worldToChassis;
    Transform<Chassis, Turret> chassisToTurret;
    Transform<World, Turret> worldToTurret;
    Transform<Turret, Chassis> turretToChassis;
    Transform<Chassis, World> chassisToWorld;

    const aruwsrc::chassis::BalancingChassisSubsystem& chassis;
    const aruwsrc::control::turret::StandardTurretSubsystem& turret;

    modm::Vector2f chassisPositionWorldFrame = modm::Vector2f(0,0);

    uint32_t prevTime = 0;
};
}  // namespace aruwsrc::balstd::transforms

#endif  // BALSTD_TRANSFORMS_HPP_