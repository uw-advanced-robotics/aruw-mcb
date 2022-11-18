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

#include "swerve_chassis_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"

#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/drivers.hpp"

#include "swerve_module.hpp"

using namespace tap::algorithms;

namespace aruwsrc
{
namespace chassis
{

SwerveChassisSubsystem::SwerveChassisSubsystem(
    aruwsrc::Drivers* drivers,
    tap::motor::MotorId leftFrontAzimuthMotorId,
    tap::motor::MotorId leftFrontDriveMotorId,
    tap::motor::MotorId leftBackAzimuthMotorId,
    tap::motor::MotorId leftBackDriveMotorId,
    tap::motor::MotorId rightFrontAzimuthMotorId,
    tap::motor::MotorId rightFrontDriveMotorId,
    tap::motor::MotorId rightBackAzimuthMotorId,
    tap::motor::MotorId rightBackDriveMotorId,
    chassis::SwerveModuleConfig config,
    tap::gpio::Analog::Pin currentPin)
    : HolonomicChassisSubsystem(drivers, currentPin),
    modules({
        SwerveModule(drivers, leftFrontDriveMotorId, leftFrontAzimuthMotorId, config),
        SwerveModule(drivers, leftBackDriveMotorId, leftBackAzimuthMotorId, config),
        SwerveModule(drivers, rightFrontDriveMotorId, rightFrontAzimuthMotorId, config),
        SwerveModule(drivers, rightBackDriveMotorId, rightBackAzimuthMotorId, config),
    })
{
    //
}


void SwerveChassisSubsystem::swerveDriveCalculate(float x, float y, float r, float maxWheelSpeed)
{
    // this is the distance between the center of the chassis to the wheel
    float chassisRotationRatio = sqrtf(
        powf(WIDTH_BETWEEN_WHEELS_X / 2.0f, 2.0f) + powf(WIDTH_BETWEEN_WHEELS_Y / 2.0f, 2.0f));

    // to take into account the location of the turret so we rotate around the turret rather
    // than the center of the chassis, we calculate the offset and than multiply however
    // much we want to rotate by
    float leftFrontRotationRatio =
        modm::toRadian(chassisRotationRatio - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    float rightFrontRotationRatio =
        modm::toRadian(chassisRotationRatio - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);
    float leftBackRotationRatio =
        modm::toRadian(chassisRotationRatio + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    float rightBackRotationRatio =
        modm::toRadian(chassisRotationRatio + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);

    
    
}


}

}