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

#ifndef CHASSIS_REL_DRIVE_HPP_
#define CHASSIS_REL_DRIVE_HPP_

#include "tap/drivers.hpp"

#include "aruwsrc/robot/control_operator_interface.hpp"

namespace aruwsrc::chassis
{
class ChassisSubsystem;

/**
 * A helper object that performs the computations necessary for chassis relative driving.
 * Call the function of the object in a chassis command's "execute" phase.
 * This will be used if you want to drive independently of the turret.
 */
class ChassisRelDrive
{
public:
    static void computeDesiredUserTranslation(
        aruwsrc::control::ControlOperatorInterface *operatorInterface,
        tap::Drivers *drivers,
        ChassisSubsystem *chassis,
        float chassisRotation,
        float *chassisXDesiredWheelspeed,
        float *chassisYDesiredWheelspeed);

    static void onExecute(
        aruwsrc::control::ControlOperatorInterface *operatorInterface,
        tap::Drivers *drivers,
        ChassisSubsystem *chassis);
};
}  // namespace aruwsrc::chassis

#endif  // CHASSIS_REL_DRIVE_HPP_
