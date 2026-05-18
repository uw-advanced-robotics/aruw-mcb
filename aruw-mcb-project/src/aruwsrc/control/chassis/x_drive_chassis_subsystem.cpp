/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "x_drive_chassis_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"

#include "holonomic_4_motor_chassis_subsystem.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::chassis
{
XDriveChassisSubsystem::XDriveChassisSubsystem(
    tap::Drivers* drivers,
    tap::communication::sensors::current::CurrentSensorInterface* currentSensor,
    tap::communication::sensors::voltage::VoltageSensorInterface* voltageSensor,
    Motor& leftFrontMotor,
    Motor& leftBackMotor,
    Motor& rightFrontMotor,
    Motor& rightBackMotor,
    tap::algorithms::SmoothPidConfig wheelVelocityPidConfig,
    float wheelRadius,
    float wheelbaseRadius,
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
    wheelVelToChassisVelMat[X][LF] = M_SQRT2;
    wheelVelToChassisVelMat[X][RF] = -M_SQRT2;
    wheelVelToChassisVelMat[X][LB] = M_SQRT2;
    wheelVelToChassisVelMat[X][RB] = -M_SQRT2;
    wheelVelToChassisVelMat[Y][LF] = -M_SQRT2;
    wheelVelToChassisVelMat[Y][RF] = -M_SQRT2;
    wheelVelToChassisVelMat[Y][LB] = M_SQRT2;
    wheelVelToChassisVelMat[Y][RB] = M_SQRT2;
    wheelVelToChassisVelMat[R][LF] = -1.0 / wheelbaseRadius;
    wheelVelToChassisVelMat[R][RF] = -1.0 / wheelbaseRadius;
    wheelVelToChassisVelMat[R][LB] = -1.0 / wheelbaseRadius;
    wheelVelToChassisVelMat[R][RB] = -1.0 / wheelbaseRadius;
    wheelVelToChassisVelMat *= (wheelRadius / 4);
}

}  // namespace aruwsrc::control::chassis