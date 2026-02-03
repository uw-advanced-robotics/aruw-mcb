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

namespace aruwsrc::control::chassis
{
MecanumChassisSubsystem::MecanumChassisSubsystem(
    tap::Drivers* drivers,
    tap::communication::sensors::current::CurrentSensorInterface* currentSensor,
    tap::communication::sensors::voltage::VoltageSensorInterface* voltageSensor,
    Motor& leftFrontMotor,
    Motor& leftBackMotor,
    Motor& rightFrontMotor,
    Motor& rightBackMotor,
    tap::algorithms::SmoothPidConfig wheelVelocityPidConfig,
    float wheelRadius,
    float effectiveWheelbase,
    communication::can::cap_bank::CapacitorBank* capacitorBank)
    : Holonomic4MotorChassisSubsystem(
          drivers,
          currentSensor,
          voltageSensor,
          leftFrontMotor,
          leftBackMotor,
          rightFrontMotor,
          rightBackMotor,
          wheelVelocityPidConfig,
          capacitorBank)
{
    wheelVelToChassisVelMat[X][LF] = 1;
    wheelVelToChassisVelMat[X][RF] = -1;
    wheelVelToChassisVelMat[X][LB] = 1;
    wheelVelToChassisVelMat[X][RB] = -1;
    wheelVelToChassisVelMat[Y][LF] = -1;
    wheelVelToChassisVelMat[Y][RF] = -1;
    wheelVelToChassisVelMat[Y][LB] = 1;
    wheelVelToChassisVelMat[Y][RB] = 1;
    wheelVelToChassisVelMat[R][LF] = -1.0 / effectiveWheelbase;
    wheelVelToChassisVelMat[R][RF] = -1.0 / effectiveWheelbase;
    wheelVelToChassisVelMat[R][LB] = -1.0 / effectiveWheelbase;
    wheelVelToChassisVelMat[R][RB] = -1.0 / effectiveWheelbase;
    wheelVelToChassisVelMat *= (wheelRadius / 4);

    // HAVENT LOOKED AT THIS YET
    chassisVelToWheelVelMat[LF][X] = 1;
    chassisVelToWheelVelMat[RF][X] = 1;
    chassisVelToWheelVelMat[LB][X] = 1;
    chassisVelToWheelVelMat[RB][X] = 1;
    chassisVelToWheelVelMat[LF][Y] = -1;
    chassisVelToWheelVelMat[RF][Y] = 1;
    chassisVelToWheelVelMat[LB][Y] = 1;
    chassisVelToWheelVelMat[RB][Y] = -1;
    chassisVelToWheelVelMat[LF][R] = -(effectiveWheelbase - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    chassisVelToWheelVelMat[RF][R] = (effectiveWheelbase - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);
    chassisVelToWheelVelMat[LB][R] = -(effectiveWheelbase + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    chassisVelToWheelVelMat[RB][R] = (effectiveWheelbase + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);
    wheelVelToChassisVelMat /= wheelRadius;
}

}  // namespace aruwsrc::control::chassis
