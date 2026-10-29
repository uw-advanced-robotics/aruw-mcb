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

#ifndef TURRET_DYNAMIC_COM_GRAVITY_COMPENSATOR_HPP_
#define TURRET_DYNAMIC_COM_GRAVITY_COMPENSATOR_HPP_

#include "tap/algorithms/transforms/transform.hpp"

#include "aruwsrc/algorithms/point_mass.hpp"

#include "turret_compensator_interface.hpp"

namespace aruwsrc::control::turret::algorithms
{
class TurretDynamicCOMGravityCompensator : public TurretCompensatorInterface
{
public:
    /**
     * @brief Configuration parameters for TurretDynamicCOMGravityCompensator.
     */
    struct TurretGravityParams
    {
        /**
         * @brief Center of gravity relative to the turret's pitch pivot in the X direction.
         *
         * The "X" direction lies along the plane the turret is pointing.
         * Units: millimeters. Positive is forward, negative is backward.
         */
        const aruwsrc::algorithms::PointMass& pointMass;

        const tap::algorithms::transforms::Transform& worldToTurretPitch;

        float motorTorqueConstant;
    };

    TurretDynamicCOMGravityCompensator(const TurretGravityParams& params);

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

#endif  // TURRET_DYNAMIC_COM_GRAVITY_COMPENSATOR_HPP_
