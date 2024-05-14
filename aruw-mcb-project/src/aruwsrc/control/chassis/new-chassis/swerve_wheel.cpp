/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "swerve_wheel.hpp"
using namespace tap::algorithms;
namespace aruwsrc
{
namespace chassis
{
SwerveWheel::SwerveWheel(
    Motor& driveMotor,
    Motor& azimuthMotor,
    const WheelConfig& config,
    SwerveAzimuthConfig& azimuthConfig,
    SmoothPid drivePid,
    SmoothPid azimuthPid)
    : Wheel(config),
      driveMotor(driveMotor),
      azimuthMotor(azimuthMotor),
      azimuthConfig(azimuthConfig),
      drivePid(drivePid),
      azimuthPid(azimuthPid),
      rotationVectorX(-config.wheelPositionChassisRelativeY),
      rotationVectorY(config.wheelPositionChassisRelativeX),
      angularBiasLUTInterpolator(
          azimuthConfig.angular_power_frac_LUT,
          MODM_ARRAY_SIZE(azimuthConfig.angular_power_frac_LUT))
{
    rotationSetpoint = 0;
    speedSetpointRPM = 0;
}

void SwerveWheel::executeWheelVelocity(float vx, float vy)
{
    moveVectorX = vx;
    moveVectorY = vy;

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

        // normal angle wrapping
        if (abs(newRotationSetpointRadians - preScaledRotationSetpoint) > M_PI)
        {
            rotationOffset -=
                getSign(newRotationSetpointRadians - preScaledRotationSetpoint) * M_TWOPI;
        }
        newRotationSetpointRadians = newRawRotationSetpointRadians + rotationOffset;

        // TODO: mechanical problem with the tension wheels in swerve module make this not work
        //       re-enable once fixed
        // reverse module if it's a smaller azimuth rotation to do so
        // TODO 2: Test again on this year's bot (2024)
        // if (abs(newRotationSetpointRadians - preScaledRotationSetpoint) > M_PI_2)
        // {
        //     rotationOffset -=
        //         getSign(newRotationSetpointRadians - preScaledRotationSetpoint) * M_PI;
        // }
        preScaledRotationSetpoint = newRawRotationSetpointRadians + rotationOffset;

        preScaledSpeedSetpoint =
            mpsToRpm(sqrtf(moveVectorX * moveVectorX + moveVectorY * moveVectorY));

        // if offset isn't an integer multiple of 2pi, it means module is currently reversed so
        // speed must be negative
        //  compareFloatClose may or may not be necessary
        if (compareFloatClose(wrapAngle(rotationOffset, M_TWOPI), M_PI, 0.1))
            preScaledSpeedSetpoint *= -1;
    }
    speedSetpointRPM = preScaledSpeedSetpoint;
    rotationSetpoint = preScaledRotationSetpoint;
}

void SwerveWheel::initialize()
{
    driveMotor.initialize();
    azimuthMotor.initialize();
}

void SwerveWheel::limitPower(float powerLimitFrac)
{
    drivePowerLimitFrac =
        powerLimitFrac * angularBiasLUTInterpolator.interpolate(rotationSetpoint - getAngle());
    azimuthPowerLimitFrac =
        powerLimitFrac *
        (1 - angularBiasLUTInterpolator.interpolate(rotationSetpoint - getAngle()));
}

void SwerveWheel::refresh()
{
    driveMotor.setDesiredOutput(
        drivePowerLimitFrac *
        drivePid.runControllerDerivateError(speedSetpointRPM - getDriveRPM(), 2.0f));
    azimuthMotor.setDesiredOutput(
        azimuthPowerLimitFrac *
        azimuthPid.runController(rotationSetpoint - getAngle(), getAngularVelocity(), 2.0f));
}

void SwerveWheel::setZeroRPM()
{
    driveMotor.setDesiredOutput(0.0f);
    azimuthMotor.setDesiredOutput(0.0f);
}

bool SwerveWheel::allMotorsOnline() const
{
    return driveMotor.isMotorOnline() && azimuthMotor.isMotorOnline();
}

float SwerveWheel::getDriveVelocity() const { return rpmToMps(driveMotor.getShaftRPM()); }

float SwerveWheel::getDriveRPM() const { return driveMotor.getShaftRPM(); }

int SwerveWheel::getNumMotors() const { return 2; }

float SwerveWheel::getAngle() const
{
    return modm::toRadian(
        azimuthMotor.encoderToDegrees(
            azimuthMotor.getEncoderUnwrapped() - azimuthConfig.azimuthZeroOffset) *
        azimuthConfig.azimuthMotorGearing);
}

std::vector<float> SwerveWheel::getMMat(){
    float r = config.diameter / 2;
    float omega = getDriveVelocity();
    float beta = getAngle();
    float xDot = r * omega * cos(beta);
    float yDot = r * omega * sin(beta);
    return std::vector<float>({xDot, yDot});
}

std::vector<float> SwerveWheel::getHMat()
{
    return std::vector<float>({
        0,
        0,
        0,
        Wheel::distanceMat.data[0],
        Wheel::distanceMat.data[1],
        Wheel::distanceMat.data[2],
        0,
        0,
        0,
        Wheel::distanceMat.data[3],
        Wheel::distanceMat.data[4],
        Wheel::distanceMat.data[5]});
}

float SwerveWheel::getAngularVelocity() const
{
    return 6.0f * static_cast<float>(azimuthMotor.getShaftRPM()) *
           (azimuthConfig.azimuthMotorGearing);
}



}  // namespace chassis

}  // namespace aruwsrc
