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

#include "swerve_module.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "aruwsrc/drivers.hpp"
#include "modm/math/geometry/angle.hpp"

using namespace tap::algorithms;

namespace aruwsrc
{
namespace chassis
{

SwerveModule::SwerveModule(
    aruwsrc::Drivers* drivers,
    tap::motor::MotorId driveMotorId,
    tap::motor::MotorId azimuthMotorId,
    SwerveModuleConfig& swerveModuleConfig)
    : driveMotor(
          drivers,
          driveMotorId,
          CAN_BUS_MOTORS,
          swerveModuleConfig.driveMotorInverted,
          "Drive motor"),
      azimuthMotor(
          drivers,
          azimuthMotorId,
          CAN_BUS_MOTORS,
          swerveModuleConfig.azimuthMotorInverted,
          "Azimuth motor"),
      config(swerveModuleConfig),
      drivePid(
          swerveModuleConfig.drivePidKp,
          swerveModuleConfig.drivePidKi,
          swerveModuleConfig.drivePidKd,
          swerveModuleConfig.drivePidMaxIntegralErrorSum,
          swerveModuleConfig.drivePidMaxOutput),
      azimuthPid(
          swerveModuleConfig.azimuthPidKp,
          swerveModuleConfig.azimuthPidKi,
          swerveModuleConfig.azimuthPidKd,
          swerveModuleConfig.azimuthPidMaxIntegralErrorSum,
          swerveModuleConfig.azimuthPidMaxOutput)
{
    rotationSetpoint = 0;
}

void SwerveModule::intialize()
{
    driveMotor.initialize();
    azimuthMotor.initialize();
}

void SwerveModule::setDesiredState(float metersPerSecond, float radianTarget)
{
    if (abs(radianTarget - getAngle()) > M_PI_2)
    {
        metersPerSecond = -metersPerSecond;
        radianTarget += M_PI;
    }
    speedSetpoint = metersPerSecond;
    rotationSetpoint = radianTarget;
}

/**
 * Compares current mps to desired mps and adds that adjustment to previous goalSetpoint
 * Compares current degree to desired degree and updates based on that 
 */
void SwerveModule::refresh()
{
    drivePid.update(speedSetpoint - getVelocity());
    driveMotor.setDesiredOutput(drivePid.getValue());

    azimuthPid.update(rotationSetpoint - getAngle());
    azimuthMotor.setDesiredOutput(azimuthPid.getValue());
}

/**
 * Returns MPS of the wheel
 */
float SwerveModule::getVelocity()
{
    float currentMotorRPM = driveMotor.getShaftRPM();
    float wheelMPS = rpmToMps(currentMotorRPM);
    return wheelMPS;
}

/**
 * This returns Radian position of motor, CCW+
 */
float SwerveModule::getAngle()
{
    float motorEncoderPositionDegree =
        azimuthMotor.encoderToDegrees(azimuthMotor.getEncoderWrapped());
    return modm::toRadian(motorEncoderPositionDegree / config.azimuthMotorGearing);
}

float SwerveModule::mpsToRpm(float mps)
{
    float SEC_PER_M = 60.0f;
    return (mps / config.WHEEL_CIRCUMFRENCE_M) * SEC_PER_M * config.driveMotorGearing;
}

float SwerveModule::rpmToMps(float rpm)
{
    float SEC_PER_M = 60.0f;
    return rpm / SEC_PER_M / config.driveMotorGearing * config.WHEEL_CIRCUMFRENCE_M;
}

}  // namespace chassis
}  // namespace aruwsrc