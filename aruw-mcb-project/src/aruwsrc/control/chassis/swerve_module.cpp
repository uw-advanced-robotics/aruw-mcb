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

using namespace tap::algorithms;

namespace aruwsrc
{
namespace chassis
{
SwerveModule::SwerveModule(Motor& driveMotor, Motor& azimuthMotor, SwerveModuleConfig& config)
    : wheel(config.WHEEL_DIAMETER_M, config.driveMotorGearing, CHASSIS_GEARBOX_RATIO),
      driveMotor(driveMotor),
      azimuthMotor(azimuthMotor),
      config(config),
      drivePid(config.drivePidConfig),
      azimuthPid(config.azimuthPidConfig),
      rotationVectorX(-config.positionWithinChassisY),
      rotationVectorY(config.positionWithinChassisX),
      angularBiasLUTInterpolator(
          config.ANGULAR_POWER_FRAC_LUT,
          MODM_ARRAY_SIZE(config.ANGULAR_POWER_FRAC_LUT))
{
    rotationSetpoint = 0;
    speedSetpointRPM = 0;
}

void SwerveModule::initialize()
{
    driveMotor.initialize();
    azimuthMotor.initialize();
}

void SwerveModule::setZeroRPM() { speedSetpointRPM = 0; }

bool SwerveModule::allMotorsOnline() const
{
    return driveMotor.isMotorOnline() && azimuthMotor.isMotorOnline();
}

float SwerveModule::calculate(float x, float y, float r)
{
    moveVectorX = x + r * rotationVectorX;
    moveVectorY = y + r * rotationVectorY;

    if (compareFloatClose(0.0f, moveVectorX, 1E-1) && compareFloatClose(0.0f, moveVectorY, 1E-1))
    {
        // deadzone set to ±0.1m/s to substitute for non-existant joystick input
        // debounce/lowpass (module does unnecessary 360s when deadzone is ±0.01m/s)
        preScaledSpeedSetpoint = 0;
    }
    else
    {
        newRawRotationSetpointRadians = atan2f(moveVectorY, moveVectorX);
        newRotationSetpointRadians = newRawRotationSetpointRadians + rotationOffset;

        // normal angle unwrapping
        if (abs(newRotationSetpointRadians - preScaledRotationSetpoint) > M_PI)
        {
            rotationOffset -=
                getSign(newRotationSetpointRadians - preScaledRotationSetpoint) * M_TWOPI;
        }
        newRotationSetpointRadians = newRawRotationSetpointRadians + rotationOffset;

        // reverse module if it's a smaller azimuth rotation to do so
        if (abs(newRotationSetpointRadians - preScaledRotationSetpoint) > M_PI_2)
        {
            rotationOffset -=
                getSign(newRotationSetpointRadians - preScaledRotationSetpoint) * M_PI;
        }
        preScaledRotationSetpoint = newRawRotationSetpointRadians + rotationOffset;

        preScaledSpeedSetpoint =
            wheel.mpsToRpm(sqrtf(moveVectorX * moveVectorX + moveVectorY * moveVectorY));

        // if offset isn't an integer multiple of 2pi, it means module is currently reversed so
        // speed must be negative
        //  compareFloatClose may or may not be necessary
        if (compareFloatClose(wrapAngle(rotationOffset, M_TWOPI), M_PI, 0.1))
            preScaledSpeedSetpoint *= -1;
    }
    return preScaledSpeedSetpoint;
}

void SwerveModule::scaleAndSetDesiredState(float scaleCoeff)
{
    setDesiredState(scaleCoeff * preScaledSpeedSetpoint, preScaledRotationSetpoint);
}

void SwerveModule::setDesiredState(float driveRpm, float radianTarget)
{
    speedSetpointRPM = driveRpm;
    rotationSetpoint = radianTarget;
}

void SwerveModule::refresh()
{
    drivePid.runControllerDerivateError(speedSetpointRPM - getDriveRPM(), 2.0f);

    azimuthPid.runController(rotationSetpoint - getAngle(), getAngularVelocity(), 2.0f);

    // updateMotorOutputs(drivePid.getOutput(), azimuthPid.getOutput());
    driveDesiredOutput = drivePid.getOutput();
    azimuthDesiredOutput = azimuthPid.getOutput();
}

void SwerveModule::updateMotorOutputs(float driveOutput, float azimuthOutput)
{
    driveDesiredOutput = driveOutput;
    driveMotor.setDesiredOutput(driveDesiredOutput);

    azimuthDesiredOutput = azimuthOutput;
    azimuthMotor.setDesiredOutput(azimuthDesiredOutput);
}

float SwerveModule::getDriveVelocity() const { return wheel.rpmToMps(driveMotor.getShaftRPM()); }

float SwerveModule::getDriveRPM() const { return driveMotor.getShaftRPM(); }

float SwerveModule::getAngle() const
{
    return modm::toRadian(
        azimuthMotor.encoderToDegrees(
            azimuthMotor.getEncoderUnwrapped() - config.azimuthZeroOffset) *
        config.azimuthMotorGearing);
}

float SwerveModule::getAngularVelocity() const
{
    return 6.0f * static_cast<float>(azimuthMotor.getShaftRPM()) * config.azimuthMotorGearing;
}

void SwerveModule::limitPower(float frac)
{
    updateMotorOutputs(driveDesiredOutput * (frac * frac), azimuthDesiredOutput * fmin(2.0f * frac , 1.0f));
    // driveMotor.setDesiredOutput(
    //     driveDesiredOutput * (frac * frac));// *
        // (1 - angularBiasLUTInterpolator.interpolate(fabs(rotationSetpoint - getAngle()))));
    // azimuthMotor.setDesiredOutput(
    //     azimuthDesiredOutput);// * fmin(2 , 1));// *
    //     (angularBiasLUTInterpolator.interpolate(fabs(rotationSetpoint - getAngle()))));
}

}  // namespace chassis
}  // namespace aruwsrc
