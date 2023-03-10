/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

/*
 * Copyright (c) 2019 Sanger_X
 */

#ifndef X_DRIVE_CHASSIS_SUBSYSTEM_HPP_
#define X_DRIVE_CHASSIS_SUBSYSTEM_HPP_

#include "tap/communication/gpio/analog.hpp"
#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/drivers.hpp"

#include "constants/chassis_constants.hpp"

#include "holonomic_4_motor_chassis_subsystem.hpp"

namespace aruwsrc
{
namespace chassis
{
/**
 * Encapsulates a chassis with omni wheels in X layout
 */
class XDriveChassisSubsystem : public Holonomic4MotorChassisSubsystem
{
public:
    XDriveChassisSubsystem(
        tap::Drivers* drivers,
        tap::motor::DjiMotor&,
        tap::motor::DjiMotor&,
        tap::motor::DjiMotor&,
        tap::motor::DjiMotor&,
        tap::gpio::Analog::Pin currentPin = CURRENT_SENSOR_PIN);
};

}  // namespace chassis

}  // namespace aruwsrc
#endif