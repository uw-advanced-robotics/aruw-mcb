/*
 * Copyright (c) 2020-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef THIRD_ORDER_COMPENSATION_HPP_
#define THIRD_ORDER_COMPENSATION_HPP_

#include <cmath>
#include <cstdint>

#include "tap/algorithms/transforms/transform.hpp"

#include "modm/math/geometry/angle.hpp"

#include "turret_compensator_interface.hpp"

namespace aruwsrc::control::turret::algorithms
{
class TurretThirdOrderCompensation : public TurretCompensatorInterface
{
public:
    /**
     * @brief Configuration parameters for TurretThirdOrderCompensation.
     */
    struct TurretThirdOrderCompensationParams
    {
        float bias;
        float firstCoefficient;
        float secondCoefficient;
        float thirdCoefficient;
    };
    /**
     * Third order polynomial compensator
     *
     * @param[in] True if the motor direction should be inverted.
     */
    TurretThirdOrderCompensation(
        const TurretThirdOrderCompensationParams& params,
        const bool isMotorInverted)
        : params(params),
          isMotorInverted(isMotorInverted){};

    /**
     * @param[in] state The state of the turret, including the pitch in world and chassis
     * frame.
     * @return The compensation effort calculated using a second order model of the turret's
     * dynamics, with the form: compensationEffort = bias + firstCoefficient * pitch +
     * secondCoefficient * pitch^2 + thirdCoefficient * pitch^3
     */
    float calculateCompensationEffort(const TurretCompensatorState state) const override
    {
        const float pitch = state.pitchChassisFrame < M_PI ? state.pitchChassisFrame + M_TWOPI
                                                           : state.pitchChassisFrame;
        const float compensationEffort = params.bias + params.firstCoefficient * pitch +
                                         params.secondCoefficient * pitch * pitch +
                                         params.thirdCoefficient * pitch * pitch * pitch;
        return isMotorInverted ? -compensationEffort : compensationEffort;
    }

private:
    const TurretThirdOrderCompensationParams params;
    const bool isMotorInverted;
};
}  // namespace aruwsrc::control::turret::algorithms

#endif  // THIRD_ORDER_COMPENSATION_HPP_