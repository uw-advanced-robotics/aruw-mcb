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

#ifndef MECANUM_CHASSIS_SUBSYSTEM_HPP_
#define MECANUM_CHASSIS_SUBSYSTEM_HPP_

#include "tap/communication/gpio/analog.hpp"
#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/drivers.hpp"

#include "constants/chassis_constants.hpp"

#include "holonomic_4_motor_chassis_subsystem.hpp"

namespace aruwsrc::control::chassis
{
/**
 * Encapsulates a chassis with mecanum wheels in standard layout
 */
class MecanumChassisSubsystem : public Holonomic4MotorChassisSubsystem
{
public:
    MecanumChassisSubsystem(
        tap::Drivers* drivers,
        tap::communication::sensors::current::CurrentSensorInterface* currentSensor,
        tap::communication::sensors::voltage::VoltageSensorInterface* voltageSensor,
        Motor& leftFrontMotor,
        Motor& leftBackMotor,
        Motor& rightFrontMotor,
        Motor& rightBackMotor,
        tap::algorithms::SmoothPidConfig wheelVelocityPidConfig,
        communication::can::cap_bank::CapacitorBank* capacitorBank = nullptr);
};

}  // namespace aruwsrc::control::chassis

#endif