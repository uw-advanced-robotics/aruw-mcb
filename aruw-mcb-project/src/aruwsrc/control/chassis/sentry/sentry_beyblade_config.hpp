/*
 * Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef SENTRY_BEYBLADE_CONFIG_HPP_
#define SENTRY_BEYBLADE_CONFIG_HPP_

namespace aruwsrc::sentry
{
struct SentryBeybladeConfig
{
    /**
     * Fraction of max chassis speed that will be applied to rotation when beyblading
     */
    const float beybladeRotationalSpeedFractionOfMax;
    /**
     * Fraction between [0, 1], what we multiply user translational input by when beyblading.
     */
    const float beybladeTranslationalSpeedMultiplier;
    /**
     * The fraction to cut rotation speed while moving and beyblading
     */
    const float beybladeRotationalSpeedMultiplierWhenTranslating;
    /**
     * Threshold, a fraction of the maximum translational speed that is used to determine if
     * beyblade speed should be reduced (when translating at an appreciable speed beyblade speed is
     * reduced).
     */
    const float translationalSpeedThresholdMultiplierForRotationSpeedDecrease;
    /**
     * Rotational speed to update the beyblade ramp target by each iteration until final rotation
     * setpoint reached, in RPM.
     */
    const float beybladeRampRate;
};
}  // namespace aruwsrc::sentry
#endif  // SENTRY_BEYBLADE_CONFIG_HPP
