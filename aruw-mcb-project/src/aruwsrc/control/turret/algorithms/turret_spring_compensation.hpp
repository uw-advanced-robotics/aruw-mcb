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

#ifndef TURRET_SPRING_COMPENSATION_HPP_
#define TURRET_SPRING_COMPENSATION_HPP_

#include <cmath>
#include <cstdint>

#include "tap/algorithms/transforms/transform.hpp"

#include "modm/math/geometry/angle.hpp"

#include "turret_compensator_interface.hpp"

namespace aruwsrc::control::turret::algorithms
{
class TurretSpringForceOffset : public TurretCompensatorInterface
{
public:
    /**
     * @brief Configuration parameters for TurretSpringForceOffset.
     */
    struct TurretSpringParams
    {
        /**
         * @brief Mounting position of the fixed end of the spring in the X direction
         *        on the pitching side of the turret.
         *
         * The "X" direction lies along the plane the turret is pointing.
         * Units: millimeters. Positive is forward, negative is backward.
         */
        float turretPitchMountX;

        /**
         * @brief Mounting position of the fixed end of the spring in the Z direction
         *        on the pitching side of the turret.
         *
         * The "Z" direction is perpendicular to the plane the turret is pointing.
         * Units: millimeters. Positive is upward, negative is downward.
         */
        float turretPitchMountZ;

        /**
         * @brief Mounting position of the fixed end of the spring in the X direction
         *        on the non-pitching (yaw) side of the turret.
         *
         * The "X" direction lies along the plane the turret is pointing.
         * Units: millimeters. Positive is forward, negative is backward.
         */
        float turretYawMountX;

        /**
         * @brief Mounting position of the fixed end of the spring in the Z direction
         *        on the non-pitching (yaw) side of the turret.
         *
         * The "Z" direction is perpendicular to the plane the turret is pointing.
         * Units: millimeters. Positive is upward, negative is downward.
         */
        float turretYawMountZ;

        /**
         * @brief Spring constant used when calculating spring force.
         *
         * Units: force per unit distance.
         */
        float springConstant;

        /**
         * @brief Free (un-stretched) length of the spring.
         *
         * This value is subtracted from the current spring length to determine
         * the stretch that produces force.
         */
        float springFreeLength;
    };
    /**
     * @param[in] True if the motor direction should be inverted.
     */
    TurretSpringForceOffset(const TurretSpringParams& params, const bool isMotorInverted);

    /**
     * @param[in] state The state of the turret, including the pitch in world and chassis frame.
     * @return The gravitational force offset necessary to cancel out gravitational
     * force of the turret, between [-gravityCompensatorMax, gravityCompensatorMax].
     * The gravitational force offset is a function of the location of the CG and
     * the current pitch angle.
     */
    float calculateCompensationEffort(const TurretCompensatorState state) const override;

    float calculateEffectiveX(const float pitch) const;

private:
    const TurretSpringParams params;
    const bool isMotorInverted;
    tap::algorithms::transforms::Position pitchPointPosition;
    tap::algorithms::transforms::Position yawPointPosition;
};
}  // namespace aruwsrc::control::turret::algorithms

#endif  // SPRING_COMPENSATION_HPP_
