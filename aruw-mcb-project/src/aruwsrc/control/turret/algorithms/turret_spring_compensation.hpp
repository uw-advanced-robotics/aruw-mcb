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

#ifndef TURRET_GRAVITY_COMPENSATION_HPP_
#define TURRET_GRAVITY_COMPENSATION_HPP_

#include <cmath>
#include <cstdint>

#include "tap/algorithms/transforms/transform.hpp"

#include "modm/math/geometry/angle.hpp"

#include "turret_compensator_interface.hpp"

namespace aruwsrc::control::turret::algorithms
{
class turretSpringForceOffset : public TurretCompensatorInterface
{
public:
    /**
     * @param[in] turretPitchMountX The mounting position of the fixed end of the spring on the X
     * direction. The "X" direction lies along the plane that the turret is pointing. Units in
     * millimeters. Positive is forward, negative is backwards.
     * @param[in] turretPitchMountZ The mounting position of the fixed end of the spring on the Z
     * (up/down) direction. The "Z" direction lies perpendicular to the plane that the turret is
     * pointing. Units in millimeters. Positive is upwards, negative is downwards.
     * @param[in] turretYawMountX The mounting position in the X direction of the fixed end of the
     * spring on the non-pitching side of the turret. The "X" direction lies along the plane that
     * the turret is pointing. Units in millimeters. Positive is forward, negative is backwards.
     * @param[in] turretYawMountZ The mounting position in the Z direction of the fixed end of the
     * spring on the non-pitching side of the turret. The "Z" direction lies perpendicular to the
     * plane that the turret is pointing. Units in millimeters. Positive is upwards, negative is
     * downwards.
     * @param[in] springConstant spring constant in units of force per distance to be used when
     * calculating the spring effort.
     */
    turretSpringForceOffset(
        const float turretPitchMountX,
        const float turretPitchMountZ,
        const float turretYawMountX,
        const float turretYawMountZ,
        const float springConstant,
        const float springFreeLength);

    /**
     * @param[in] state The state of the turret, including the pitch in world and chassis frame.
     * @return The gravitational force offset necessary to cancel out gravitational
     * force of the turret, between [-gravityCompensatorMax, gravityCompensatorMax].
     * The gravitational force offset is a function of the location of the CG and
     * the current pitch angle.
     */
    float calculateCompensationEffort(const TurretCompensatorState state) const override;

private:
    tap::algorithms::transforms::Position pitchPointPosition;
    tap::algorithms::transforms::Position yawPointPosition;
    const float springConstant;
    const float springFreeLength;
};
}  // namespace aruwsrc::control::turret::algorithms

#endif  // GRAVITY_COMPENSATION_HPP_
