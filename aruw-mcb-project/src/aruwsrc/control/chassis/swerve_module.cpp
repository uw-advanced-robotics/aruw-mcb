/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

using namespace tap::algorithms;

namespace aruwsrc
{
namespace chassis
{
SwerveModule::SwerveModule(
    tap::Drivers* drivers,
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
      config(config),
      drivePid(
          config.drivePidKp,
          config.drivePidKi,
          config.drivePidKd,
          config.drivePidMaxIntegralErrorSum,
          config.drivePidMaxOutput),
      azimuthPid(config.azimuthPidConfig),
      rotationVectorX(-config.positionWithinChassisY),
      rotationVectorY(config.positionWithinChassisX)
{
    rotationSetpoint = 0;
    speedSetpointRPM = 0;
}

void SwerveModule::initialize()
{
    driveMotor.initialize();
    azimuthMotor.initialize();
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
    //probably a naive way of doing this but haven't thought of a better one yet
    //  (comparing power used for maintaining position vs velocity)
    return ANGULAR_ERROR_POWER_BIAS * getAzimuthError() + getDriveError();
}


float SwerveModule::getAzimuthError() const
{
    return 0;//azimuthPid.getLastError();  //**FIX THIS SMOOTHPID DOESNT HAVE GETLASTERROR IDK WHAT TO DO**
}

//
float SwerveModule::getDriveError() const
{
    return drivePid.getLastError();
}

float SwerveModule::calculate(float x, float y, float r)
{
    float moveVectorX = x + r * rotationVectorX;
    float moveVectorY = y + r * rotationVectorY;

    if(compareFloatClose(0.0f, moveVectorX, 1E-1) && compareFloatClose(0.0f, moveVectorY, 1E-1))
    {
        //deadzone (temporarily?) set to ±0.1m/s to substitute for non-existant joystick input debounce/lowpass
        //  (module did unnecessary 360s when deadzone was ±0.01m/s)
        preScaledSpeedSetpoint = 0;
    }
    else
    {
        float newRawRotationSetpointRadians = atan2f(moveVectorY, moveVectorX);
        float newRotationSetpointRadians = newRawRotationSetpointRadians + rotationOffset;
        
        //normal angle wrapping
        if (abs(newRotationSetpointRadians - rotationSetpointRadians) > M_PI)
        {
            rotationOffset -= getSign(newRotationSetpointRadians - rotationSetpointRadians) * M_TWOPI;//TWOPI == 2*PI
        }
        newRotationSetpointRadians = newRawRotationSetpointRadians + rotationOffset;
        
        //reverse module if it's a smaller azimuth rotation to do so
        if(abs(newRotationSetpointRadians - rotationSetpointRadians) > M_PI_2)
        {
            rotationOffset -= getSign(newRotationSetpointRadians - rotationSetpointRadians) * M_PI;
        }
        rotationSetpointRadians = newRawRotationSetpointRadians + rotationOffset;

        preScaledSpeedSetpoint = mpsToRpm(sqrtf(moveVectorX*moveVectorX + moveVectorY*moveVectorY));

        //if offset isn't an integer multiple of 2pi, it means module is currently reversed so speed must be negative
        //  compareFloatClose may or may not be necessary
        if(compareFloatClose(wrapAngle(rotationOffset, M_TWOPI), M_PI, 0.1)) preScaledSpeedSetpoint *= -1;
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

void SwerveModule::refresh()
{
    drivePid.update(speedSetpointRPM - getDriveRPM());
    driveMotor.setDesiredOutput(drivePid.getValue());

    azimuthPid.runController(
            rotationSetpoint - getAngle(),
            getAngularVelocity(),
            2.0f);
    azimuthMotor.setDesiredOutput(azimuthPid.getOutput());
}

float SwerveModule::getDriveVelocity() const
{
    return rpmToMps(driveMotor.getShaftRPM());
}

float SwerveModule::getDriveRPM() const
{
    return driveMotor.getShaftRPM();
}

float SwerveModule::getAngle() const
{
    return modm::toRadian(azimuthMotor.encoderToDegrees(azimuthMotor.getEncoderUnwrapped() 
        - config.azimuthZeroOffset) * config.azimuthMotorGearing);//azimuthZeroOffset temporarily replaced w 0
}

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

void SwerveModule::limitPower(float frac)
{
    driveMotor.setDesiredOutput(driveMotor.getOutputDesired() * frac);
    azimuthMotor.setDesiredOutput(azimuthMotor.getOutputDesired() * frac);
}

}  // namespace chassis
}  // namespace aruwsrc