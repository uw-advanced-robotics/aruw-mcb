/*
 * Copyright (c) 2020-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

/**
 * @file gravity_autotune.hpp
 *
 * @brief   Implements gravity-based center-of-mass autotuning for turret calibration.
 *
 * Defines the GravityAutotuneCommand command, which locks the turret at specified
 * test points, measures torque/angle, and estimates the turret's center of
 * mass using least squares regression.
 */

#ifndef AUTOTUNE_COMMAND_INTERFACE_HPP_
#define AUTOTUNE_COMMAND_INTERFACE_HPP_

#include "tap/control/command.hpp"

namespace aruwsrc::control::autotune
{
/** @brief Non-template class to allow for getting the gravity autotune commands
 * in a weak function, as used in the gravity autotune menu.
 */
class TurretAutotuneInterface : public tap::control::Command
{
public:
    enum class CalibrationState
    {
        WAITING_FOR_SYSTEMS_ONLINE,
        LOCKING_TURRET,
        MEASURING_TORQUE,
        NEXT_LOCATION,
        CALIBRATION_SUCCESS,
        CALIBRATION_FAIL,
        DONE
    };

    virtual ~TurretAutotuneInterface() = default;

    virtual CalibrationState getCalibrationState() const = 0;

    virtual const char *getName() const = 0;

    virtual void drawCalibrationResult(modm::GraphicDisplay &display) const = 0;
};

}  // namespace aruwsrc::control::autotune

#endif  // AUTOTUNE_COMMAND_INTERFACE_HPP_