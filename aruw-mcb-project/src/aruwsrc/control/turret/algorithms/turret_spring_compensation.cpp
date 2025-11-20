/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "turret_spring_compensation.hpp"

using namespace aruwsrc::control::turret::algorithms;
using namespace tap::algorithms::transforms;

TurretSpringForceOffset::TurretSpringForceOffset(
    const float turretPitchMountX,
    const float turretPitchMountZ,
    const float turretYawMountX,
    const float turretYawMountZ,
    const float springConstant,
    const float springFreeLength)
    : pitchPointPosition(turretPitchMountX, 0, turretPitchMountZ),
      yawPointPosition(turretYawMountX, 0, turretYawMountZ),
      springConstant(springConstant),
      springFreeLength(springFreeLength){};

float TurretSpringForceOffset::calculateCompensationEffort(const TurretCompensatorState state) const
{
    // apply rotation to pitch
    const Transform pitchTransform(0, 0, 0, 0, state.pitchChassisFrame, 0);

    const Position pitchPoint = pitchTransform.apply(pitchPointPosition);

    const float spring_dist = Position::distance(pitchPoint, yawPointPosition);
    const float x = spring_dist - springFreeLength;
    return x * springConstant;
}