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
#include "turret_dynamic_com_gravity_compensator.hpp"

#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::algorithms::transforms;
using tap::algorithms::ACCELERATION_GRAVITY;

namespace aruwsrc::control::turret::algorithms
{
TurretDynamicCOMGravityCompensator::TurretDynamicCOMGravityCompensator(
    const TurretGravityParams& params)
    : params(params){};

float TurretDynamicCOMGravityCompensator::calculateCompensationEffort(TurretCompensatorState) const
{
    Position pitchToCOM = params.worldToTurretPitch.apply(params.pointMass.location);
    Vector turretPitchRelativeGravityForce = params.worldToTurretPitch.apply(
        Vector(0, 0, -ACCELERATION_GRAVITY * params.pointMass.mass));
    Vector turretPitchRelativeGravityTorque =
        pitchToCOM.toVector().cross(params.worldToTurretPitch.apply(
            Vector(0, 0, -ACCELERATION_GRAVITY * params.pointMass.mass)));
    return -turretPitchRelativeGravityTorque.y() * params.motorTorqueConstant;
};

}  // namespace aruwsrc::control::turret::algorithms
