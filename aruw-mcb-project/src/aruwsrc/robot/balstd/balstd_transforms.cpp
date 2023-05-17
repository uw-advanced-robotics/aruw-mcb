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

#include "balstd_transforms.hpp"

using namespace tap::algorithms::transforms;

namespace aruwsrc::balstd::transforms
{

Transformer::Transformer(
    const aruwsrc::chassis::BalancingChassisSubsystem& chassis,
    const aruwsrc::control::turret::StandardTurretSubsystem& turret)
    : chassis(chassis),
      turret(turret)
{
    worldToChassis = Transform<World, Chassis>();
    chassisToTurret = Transform<Chassis, Turret>(0, 0, 0, 0, 0, 0);
}

void Transformer::updateTransforms()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;
    float yaw = turret.yawMotor.getChassisFrameMeasuredAngle().getValue();
    float pitch = turret.pitchMotor.getChassisFrameMeasuredAngle().getValue();
    chassisToTurret.updateRotation(0, pitch, yaw);
    worldToTurret = compose<World, Chassis, Turret>(worldToChassis, chassisToTurret);

    modm::Vector2f chassisVelocityWorldFrame =
        modm::Vector2f(chassis.getActualVelocityChassisRelative().element[0], 0);

    tap::algorithms::rotateVector(
        &chassisVelocityWorldFrame.x,
        &chassisVelocityWorldFrame.y,
        chassis.getChassisOrientationWorldRelative().element[2]);

    chassisPositionWorldFrame += chassisVelocityWorldFrame * (dt / 1'000);

    turretToChassis = chassisToTurret.getInverse();
    chassisToWorld = worldToChassis.getInverse();
}

}  // namespace aruwsrc::balstd::transforms