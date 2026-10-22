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

#include "modm/math/geometry/angle.hpp"

#include "turret_compensator_interface.hpp"

namespace aruwsrc::control::turret::algorithms
{
class TurretGravitationalForceOffset : public TurretCompensatorInterface
{
public:
    /**
     * @brief Configuration parameters for TurretGravitationalForceOffset.
     */
    struct TurretGravityParams
    {
        /**
         * @brief Center of gravity relative to the turret's pitch pivot in the X direction.
         *
         * The "X" direction lies along the plane the turret is pointing.
         * Units: millimeters. Positive is forward, negative is backward.
         */
        float cgX;

        /**
         * @brief Center of gravity relative to the turret's pitch pivot in the Z direction.
         *
         * The "Z" direction is perpendicular to the plane the turret is pointing.
         * Units: millimeters. Positive is upward, negative is downward.
         */
        float cgZ;

        /**
         * @brief Maximum motor output value this compensator is allowed to return.
         *
         * This should correspond to the output required to fully cancel gravity when
         * the CG lies on the same XY-plane as the pivot (i.e., when gravitational torque
         * is greatest).
         */
        float gravityCompensatorMax;
    };

    TurretGravitationalForceOffset(const TurretGravityParams& params);

    /**
     * @param[in] state The state of the turret, including the pitch in world and chassis frame.
     * @return The gravitational force offset necessary to cancel out gravitational
     * force of the turret, between [-gravityCompensatorMax, gravityCompensatorMax].
     * The gravitational force offset is a function of the location of the CG and
     * the current pitch angle.
     */
    float calculateCompensationEffort(const TurretCompensatorState state) const override;

private:
    const TurretGravityParams params;
};
}  // namespace aruwsrc::control::turret::algorithms

#endif  // GRAVITY_COMPENSATION_HPP_
