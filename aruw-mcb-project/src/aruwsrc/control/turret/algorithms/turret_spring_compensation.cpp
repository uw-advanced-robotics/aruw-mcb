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

#include "tap/algorithms/math_user_utils.hpp"

using namespace aruwsrc::control::turret::algorithms;
using namespace tap::algorithms::transforms;

TurretSpringForceOffset::TurretSpringForceOffset(
    const TurretSpringParams& params,
    const bool isMotorInverted)
    : params(params),
      isMotorInverted(isMotorInverted),
      pitchPointPosition(params.turretPitchMountX, 0.0f, params.turretPitchMountZ),
      yawPointPosition(params.turretYawMountX, 0.0f, params.turretYawMountZ){};

static inline Vector normalize(const Vector vector)
{
    return vector * tap::algorithms::fastInvSqrt(Vector::dot(vector, vector));
};

float TurretSpringForceOffset::calculateCompensationEffort(const TurretCompensatorState state) const
{
    const Transform pitchTransform(0, 0, 0, 0, state.pitchChassisFrame, 0);

    const Position pitchPoint = pitchTransform.apply(pitchPointPosition);

    const float springLength = (yawPointPosition - pitchPoint).magnitude();

    const float force = (springLength - params.springFreeLength) * params.springConstant;

    const Vector springDirection = normalize(yawPointPosition - pitchPoint);

    const Vector springForceVector = springDirection * force;

    const Vector pitchPosVector(pitchPoint.coordinates());

    const Vector torque = Vector::cross(pitchPosVector, springForceVector);

    return isMotorInverted ? Vector::dot(torque, {0, 1.0, 0}) : -Vector::dot(torque, {0, 1.0, 0});
}

// Function currently used in the autotuning portion for the linear fit
float TurretSpringForceOffset::calculateEffectiveX(const float pitch) const
{
    const Transform pitchTransform(0, 0, 0, 0, pitch, 0);

    const Position pitchPoint = pitchTransform.apply(pitchPointPosition);

    const float springLength = (yawPointPosition - pitchPoint).magnitude();
    return springLength - params.springFreeLength;
}