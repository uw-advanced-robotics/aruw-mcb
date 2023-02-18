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


#include "swerve_module.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "aruwsrc/drivers.hpp"
#include "modm/math/geometry/angle.hpp"

using namespace tap::algorithms;

namespace aruwsrc
{
namespace chassis
{

// SwerveModule::SwerveModule(
//     aruwsrc::Drivers* drivers,
//     tap::motor::MotorId driveMotorId,
//     tap::motor::MotorId azimuthMotorId,
//     float positionWithinChassisX,
//     float positionWithinChassisY,
//     SwerveModuleConfig& config,
//     tap::algorithms::SmoothPidConfig azimuthPidConfig)
//     : driveMotor(
//         drivers,
//         driveMotorId,
//         CAN_BUS_MOTORS,
//         config.driveMotorInverted,
//         "Drive motor"),
//     azimuthMotor(
//         drivers,
//         azimuthMotorId,
//         CAN_BUS_MOTORS,
//         config.azimuthMotorInverted,
//         "Azimuth motor"),
//     drivePid(
//         config.drivePidKp,
//         config.drivePidKi,
//         config.drivePidKd,
//         config.drivePidMaxIntegralErrorSum,
//         config.drivePidMaxOutput),
//     azimuthPid(azimuthPidConfig),
//     config(config)
// {
//     rotationSetpoint = 0;
//     speedSetpointRPM = 0;
//     rotationVectorX = -positionWithinChassisY;
//     rotationVectorY = positionWithinChassisX;
// }

SwerveModule::SwerveModule(
    aruwsrc::Drivers* drivers,
    SwerveModuleConfig& config)
    : driveMotor(
          drivers,
          config.driveMotorId,
          CAN_BUS_MOTORS,
          config.driveMotorInverted,
          "Drive motor"),
      azimuthMotor(
          drivers,
          config.azimuthMotorId,
          CAN_BUS_MOTORS,
          config.azimuthMotorInverted,
          "Azimuth motor"),
      drivePid(
          config.drivePidKp,
          config.drivePidKi,
          config.drivePidKd,
          config.drivePidMaxIntegralErrorSum,
          config.drivePidMaxOutput),
      azimuthPid(config.azimuthPidConfig),
      config(config)
{
    rotationSetpoint = 0;
    speedSetpointRPM = 0;
    rotationVectorX = -config.positionWithinChassisY;
    rotationVectorY = config.positionWithinChassisX;
}

void SwerveModule::initialize()
{
    driveMotor.initialize();
    azimuthMotor.initialize();
}

void SwerveModule::calibrateAzimuth()
{
    //azimuthZeroOffset = azimuthMotor.getEncoderUnwrapped();
    //isCalibrated = true;
}

void SwerveModule::setZeroRPM()
{
    speedSetpointRPM = 0;
}

bool SwerveModule::allMotorsOnline() const
{
    return driveMotor.isMotorOnline() && azimuthMotor.isMotorOnline();
}

float SwerveModule::calculateTotalModuleError() const
{
    return ANGULAR_ERROR_POWER_BIAS * getAzimuthError() + getDriveError();
}


//radians
float SwerveModule::getAzimuthError() const
{
    return 0;//azimuthPid.getLastError();  //**FIX THIS SMOOTHPID DOESNT HAVE GETLASTERROR**
}

//
float SwerveModule::getDriveError() const
{
    return drivePid.getLastError();
}

/**
 * computes initial candidate for module state
 * @param x desired chassis x velocity in m/s
 * @param y desired chassis y velocity in m/s
 * @param r desired chassis angular velocity in rad/s
 * @return pre-scaled module speed
*/
float SwerveModule::calculate(float x, float y, float r)
{
    float moveVectorX = x + r * rotationVectorX;
    float moveVectorY = y + r * rotationVectorY;

    if(compareFloatClose(0.0f, moveVectorX, 1E-1) && compareFloatClose(0.0f, moveVectorY, 1E-1))
    {
        preScaledSpeedSetpoint = 0;
    }
    else
    {
        float newRawRotationSetpointRadians = atan2f(moveVectorY, moveVectorX);
        float newRotationSetpointRadians = newRawRotationSetpointRadians + rotationOffset;
        
        if (abs(newRotationSetpointRadians - rotationSetpointRadians) > M_PI)
        {
            rotationOffset -= getSign(newRotationSetpointRadians - rotationSetpointRadians) * M_TWOPI;//TWOPI == 2*PI
        }
        newRotationSetpointRadians = newRawRotationSetpointRadians + rotationOffset;
        if(abs(newRotationSetpointRadians - rotationSetpointRadians) > M_PI_2)//PI_2 == PI/2
        {
            rotationOffset -= getSign(newRotationSetpointRadians - rotationSetpointRadians) * M_PI;
            //reversed &= false;
        }
        rotationSetpointRadians = newRawRotationSetpointRadians + rotationOffset;
        preScaledSpeedSetpoint = mpsToRpm(sqrtf(moveVectorX*moveVectorX + moveVectorY*moveVectorY));
        if(compareFloatClose(unwrapAngle(rotationOffset, M_TWOPI), M_PI, 0.1)) preScaledSpeedSetpoint *= -1;
    }
    return preScaledSpeedSetpoint;
}

void SwerveModule::scaleAndSetDesiredState(float scaleCoeff)
{
    setDesiredState(scaleCoeff*preScaledSpeedSetpoint, rotationSetpointRadians);
}

void SwerveModule::setDesiredState(float driveRpm, float radianTarget)
{
    speedSetpointRPM = driveRpm;
    rotationSetpoint = radianTarget;
}

/**
 * Compares current mps to desired mps and adds that adjustment to previous goalSetpoint
 * Compares current degree to desired degree and updates based on that
 */
void SwerveModule::refresh()
{
    drivePid.update(speedSetpointRPM - getDriveRPM());
    drivePIDOutput = drivePid.getValue();
    driveMotor.setDesiredOutput(drivePIDOutput);

    azimuthPid.runController(
            rotationSetpoint - getAngle(),
            getAngularVelocity(),
            2.0f);
    azimuthMotor.setDesiredOutput(azimuthPid.getOutput());
}

/**
 * Returns MPS of the wheel
 */
float SwerveModule::getDriveVelocity() const
{
    return rpmToMps(driveMotor.getShaftRPM());
}

/**
 * Returns RPM of the wheel
 */
float SwerveModule::getDriveRPM() const
{
    return driveMotor.getShaftRPM();
}

/**
 * This returns Radian position of azimuth motor, CCW+
 */
float SwerveModule::getAngle() const
{
    return modm::toRadian(azimuthMotor.encoderToDegrees(azimuthMotor.getEncoderUnwrapped() 
        - config.azimuthZeroOffset) * config.azimuthMotorGearing);//azimuthZeroOffset temporarily replaced w 0
}

/**
 * This returns deg/sec velocity of azimuth motor, CCW+
 */
float SwerveModule::getAngularVelocity() const
{
    return 6.0f * static_cast<float>(azimuthMotor.getShaftRPM()) * config.azimuthMotorGearing;
}

float SwerveModule::mpsToRpm(float mps) const
{
    return (mps / config.WHEEL_CIRCUMFRENCE_M) / CHASSIS_GEARBOX_RATIO * 60.0f / config.driveMotorGearing;
}

float SwerveModule::rpmToMps(float rpm) const
{
    return rpm * CHASSIS_GEARBOX_RATIO / 60.0f * config.driveMotorGearing * config.WHEEL_CIRCUMFRENCE_M;
}

float SwerveModule::optimizeAngle(float desiredAngle)
{
    desiredAziWrapNum = 0;
    while (abs(desiredAngle - getAngle() > M_PI))
    {
        // minus cuz if less than you'ld want to add but this would return a negative with get sign
        desiredAngle = desiredAngle - getSign(desiredAngle - getAngle()) * M_TWOPI;//TWOPI = 2*PI
        desiredAziWrapNum++;
    }
    return desiredAngle;
}

void SwerveModule::limitPower(float frac)
{
    //TODO: is getOutputDesired what i want to do here
    driveMotor.setDesiredOutput(driveMotor.getOutputDesired() * frac);
    azimuthMotor.setDesiredOutput(azimuthMotor.getOutputDesired() * frac);
}

}  // namespace chassis
}  // namespace aruwsrc