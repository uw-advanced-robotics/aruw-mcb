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

float TurretSpringForceOffset::calculateCompensationEffort(const TurretCompensatorState state) const
{
    const float torqueY = calculateEffectiveMoment(state.pitchChassisFrame) * params.springConstant;

    return isMotorInverted ? torqueY : -torqueY;
}

// Function currently used in the autotuning portion for the linear fit and kinda as a helper
float TurretSpringForceOffset::calculateEffectiveMoment(float pitch) const
{
    // Transform is negative as .apply() tells us what the pitchPointPosition in the applied (base)
    // frame, thus we take the inverse to give us base frame to turret frame
    const Transform pitchTransformInv(0, 0, 0, 0, -pitch, 0);

    const Position pitchPoint = pitchTransformInv.apply(pitchPointPosition);

    const Vector springVector = yawPointPosition - pitchPoint;

    const float currentLength = springVector.magnitude();

    // Div-by-zero safety
    if (currentLength < 1e-6) return 0.0f;

    const Vector pitchPosVector(pitchPoint.coordinates());

    const float scaleFactor = (currentLength - params.springFreeLength) / currentLength;

    const Vector springForceVector = springVector * scaleFactor;

    const Vector torque = Vector::cross(pitchPosVector, springForceVector);

    return torque.y();
}