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

#ifndef SENTRY_CHASSIS_REL_DRIVE_HPP_
#define SENTRY_CHASSIS_REL_DRIVE_HPP_

#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/new-chassis/chassis_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_control_operator_interface.hpp"

namespace aruwsrc::sentry
{
/**
 * A helper object that performs the computations necessary for chassis relative driving.
 * Call the function of the object in a chassis command's "execute" phase.
 * This will be used if you want to drive independently of the turret.
 */
class SentryChassisRelDrive
{
public:
    static void computeDesiredUserTranslation(
        aruwsrc::control::sentry::SentryControlOperatorInterface *operatorInterface,
        tap::Drivers *drivers,
        aruwsrc::chassis::ChassisSubsystem *chassis,
        float chassisRotation,
        float *chassisXDesiredWheelspeed,
        float *chassisYDesiredWheelspeed);

    static void onExecute(
        aruwsrc::control::sentry::SentryControlOperatorInterface *operatorInterface,
        tap::Drivers *drivers,
        aruwsrc::chassis::ChassisSubsystem *chassis);
};
}  // namespace aruwsrc::sentry

#endif  // SENTRY_CHASSIS_REL_DRIVE_HPP_
