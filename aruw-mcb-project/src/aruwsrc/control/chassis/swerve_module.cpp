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
    SwerveModuleConfig& swerveModuleConfig,
    float positionWithinChassisX,
    float positionWithinChassisY)
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
          swerveModuleConfig.azimuthPidMaxOutput),
      config(swerveModuleConfig)
{
    rotationSetpoint = 0;
    speedSetpoint = 0;
    rotationVectorX = -positionWithinChassisY;
    rotationVectorY = positionWithinChassisX;
}

void SwerveModule::intialize()
{
    driveMotor.initialize();
    azimuthMotor.initialize();
}

float SwerveModule::calculateTotalModuleError()
{
    return ANGULAR_ERROR_POWER_BIAS * getAzimuthError() + getDriveError();
}


//radians
float SwerveModule::getAzimuthError()
{
    return azimuthPid.getLastError();
}

//
float SwerveModule::getDriveError()
{
    return drivePid.getLastError();
}


float SwerveModule::calculate(float x, float y, float r)
{
    float moveVectorX = x + r * rotationVectorX;
    float moveVectorY = y + r * rotationVectorY;

    if(moveVectorX==0 && moveVectorY==0)
    //todo: maybe do smart braking by keeping
    //todo: dont use exact comparison? not sure
    {
        preScaledSpeedSetpoint = 0;
    }
    else
    {
        preOptimizedRotationSetpoint = atan2f(moveVectorY, moveVectorX);
        preScaledSpeedSetpoint = sqrtf(moveVectorX*moveVectorX + moveVectorY*moveVectorY);
    }
    return preScaledSpeedSetpoint;
}

void SwerveModule::scaleAndSet(float scaleCoeff)
{
    setDesiredState(scaleCoeff*preScaledSpeedSetpoint, preOptimizedRotationSetpoint);
}

void SwerveModule::setDesiredState(float metersPerSecond, float radianTarget)
{
    radianTarget = optimizeAngle(radianTarget);
    if (abs(modm::toDegree(radianTarget - getAngle())) >= 90)
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
    drivePid.update(speedSetpoint - getDriveVelocity());
    driveMotor.setDesiredOutput(driveMotor.getOutputDesired() + drivePid.getValue());

    azimuthPid.update(rotationSetpoint - getAngle());
    azimuthMotor.setDesiredOutput(azimuthPid.getValue());
}

/**
 * Returns MPS of the wheel
 */
float SwerveModule::getDriveVelocity() const
{
    float currentMotorRPM = driveMotor.getShaftRPM();
    float wheelMPS = rpmToMps(currentMotorRPM);
    return wheelMPS;
}

/**
 * This returns Radian position of motor, CCW+
 */
float SwerveModule::getAngle() const
{
    float motorEncoderPositionDegree =
        azimuthMotor.encoderToDegrees(azimuthMotor.getEncoderUnwrapped());
    return modm::toRadian(motorEncoderPositionDegree / config.azimuthMotorGearing);
}

float SwerveModule::mpsToRpm(float mps) const
{
    return (mps / config.WHEEL_CIRCUMFRENCE_M) * 60.0f * config.driveMotorGearing;
}

float SwerveModule::rpmToMps(float rpm) const
{
    return rpm / 60.0f / config.driveMotorGearing * config.WHEEL_CIRCUMFRENCE_M;
}

float SwerveModule::optimizeAngle(float desiredAngle)
{
    desiredAngle = modm::toDegree(desiredAngle);
    float rotationScalar = modm::toDegree(getAngle()) / 360;
    desiredAngle = desiredAngle + rotationScalar * 360;
    if (abs(desiredAngle - modm::toDegree(getAngle()) > 180))
    {
        // minus cuz if less than you'ld want to add but this would return a negative with get sign
        desiredAngle = desiredAngle - getSign(desiredAngle - modm::toDegree(getAngle()) * 360);
    }
    return modm::toRadian(desiredAngle);
}

void SwerveModule::limitPower(float frac)
{
    //TODO: is getOutputDesired what i want to do here
    driveMotor.setDesiredOutput(driveMotor.getOutputDesired() * frac);
    azimuthMotor.setDesiredOutput(azimuthMotor.getOutputDesired() * frac);
}

}  // namespace chassis
}  // namespace aruwsrc