/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "drone_transformer.hpp"

#include "aruwsrc/control/client-display/projection_utils.hpp"

namespace aruwsrc::drone
{
using aruwsrc::control::client_display::VTM_OFFSET;
using tap::algorithms::transforms::Orientation;
using tap::algorithms::transforms::Position;
using tap::algorithms::transforms::Transform;

static Transform TURRET_TO_ARDUCAM_OFFSET = Transform(Position(0.048907, 0.0011831, 0.17776), Orientation(0, 0, 0));

DroneTransformer::DroneTransformer(
    const aruwsrc::control::turret::TurretOrientationInterface& turretOrientation,
    const tap::communication::sensors::imu::AbstractIMU* turretImu)
    : turretOrientation(turretOrientation),
      turretImu(turretImu),
      worldToChassis(Transform::identity()),
      worldToTurret(Transform::identity()),
      chassisToTurret(Transform::identity()),
      worldToVTM(Transform::identity()),
      chassisToArducam(Transform::identity())
{
}

void DroneTransformer::updateTransforms()
{
    const float chassisYaw = turretImu ? turretImu->getYaw() : 0.0f;
    const float chassisRoll = turretImu ? turretImu->getRoll() : 0.0f;

    worldToChassis.updateTranslation(0.0f, 0.0f, 0.0f);
    worldToChassis.updateRotation(0.0f, 0.0f, chassisYaw);

    worldToTurret.updateRotation(
        chassisRoll,
        turretOrientation.getWorldPitch(),
        turretOrientation.getWorldYaw());

    if (turretImu)
    {
        worldToTurret.updateAngularVelocity(0.0f, turretImu->getGy(), turretImu->getGz());
    }
    else
    {
        worldToTurret.updateAngularVelocity(0.0f, 0.0f, 0.0f);
    }

    const modm::Vector3f turretOffset = turretOrientation.getTurretOffset();
    worldToTurret.updateTranslation(
        worldToChassis.getX() + turretOffset.x,
        worldToChassis.getY() + turretOffset.y,
        worldToChassis.getZ() + turretOffset.z);

    chassisToTurret = worldToChassis.getInverse().composeStatic(worldToTurret);

    Transform chassisToTurretNoPitch = chassisToTurret;
    chassisToTurretNoPitch.updateRotation(Orientation(0, 0, chassisToTurret.getRotation().yaw()));
    chassisToArducam = chassisToTurretNoPitch.composeStatic(TURRET_TO_ARDUCAM_OFFSET);

    worldToVTM = worldToTurret.composeStatic(VTM_OFFSET);
}
}  // namespace aruwsrc::drone
