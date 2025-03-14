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

#include "mecanum_chassis_subsystem.hpp"

#include "tap/drivers.hpp"

#include "holonomic_chassis_subsystem.hpp"

using namespace tap::algorithms;

namespace aruwsrc
{
namespace chassis
{
MecanumChassisSubsystem::MecanumChassisSubsystem(
    tap::Drivers* drivers,
    tap::communication::sensors::current::CurrentSensorInterface* currentSensor,
    can::capbank::CapacitorBank* capacitorBank,
    tap::motor::MotorId leftFrontMotorId,
    tap::motor::MotorId leftBackMotorId,
    tap::motor::MotorId rightFrontMotorId,
    tap::motor::MotorId rightBackMotorId)
    : Holonomic4MotorChassisSubsystem(
          drivers,
          currentSensor,
          capacitorBank,
          leftFrontMotorId,
          leftBackMotorId,
          rightFrontMotorId,
          rightBackMotorId)
{
    wheelVelToChassisVelMat[X][LF] = 1;
    wheelVelToChassisVelMat[X][RF] = -1;
    wheelVelToChassisVelMat[X][LB] = 1;
    wheelVelToChassisVelMat[X][RB] = -1;
    wheelVelToChassisVelMat[Y][LF] = -1;
    wheelVelToChassisVelMat[Y][RF] = -1;
    wheelVelToChassisVelMat[Y][LB] = 1;
    wheelVelToChassisVelMat[Y][RB] = 1;
    wheelVelToChassisVelMat[R][LF] = -1.0 / WHEELBASE_HYPOTENUSE;
    wheelVelToChassisVelMat[R][RF] = -1.0 / WHEELBASE_HYPOTENUSE;
    wheelVelToChassisVelMat[R][LB] = -1.0 / WHEELBASE_HYPOTENUSE;
    wheelVelToChassisVelMat[R][RB] = -1.0 / WHEELBASE_HYPOTENUSE;
    wheelVelToChassisVelMat *= (WHEEL_RADIUS / 4);
}

}  // namespace chassis

}  // namespace aruwsrc
