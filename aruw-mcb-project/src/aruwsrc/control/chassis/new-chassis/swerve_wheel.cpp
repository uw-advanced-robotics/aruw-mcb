
#include "swerve_wheel.hpp"
using namespace tap::algorithms;
namespace aruwsrc
{
namespace chassis
{
SwerveWheel::SwerveWheel(
    Motor& driveMotor,
    Motor& azimuthMotor,
    WheelConfig& config,
    AzimuthConfig& azimuthConfig,
    SmoothPid drivePid,
    SmoothPid azimuthPid)
    : Wheel(driveMotor, config),
      azimuthConfig(azimuthConfig),
      driveMotor(driveMotor),
      azimuthMotor(azimuthMotor),
      drivePid(drivePid),
      azimuthPid(azimuthPid),
      rotationVectorX(-config.wheelPositionChassisRelativeY),
      rotationVectorY(config.wheelPositionChassisRelativeX)
{
    rotationSetpoint = 0;
    speedSetpointRPM = 0;
}


modm::Pair<float, float> SwerveWheel::calculateDesiredWheelVelocity(float vx, float vy, float vr)
{
    moveVectorX = vx + vr * rotationVectorX;
    moveVectorY = vy + vr * rotationVectorY;

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
        if (abs(newRotationSetpointRadians - preScaledRotationSetpoint) > M_PI_2)
        {
            rotationOffset -=
                getSign(newRotationSetpointRadians - preScaledRotationSetpoint) * M_PI;
        }
        preScaledRotationSetpoint = newRawRotationSetpointRadians + rotationOffset;

        preScaledSpeedSetpoint =
            mpsToRpm(sqrtf(moveVectorX * moveVectorX + moveVectorY * moveVectorY));

        // if offset isn't an integer multiple of 2pi, it means module is currently reversed so
        // speed must be negative
        //  compareFloatClose may or may not be necessary
        if (compareFloatClose(wrapAngle(rotationOffset, M_TWOPI), M_PI, 0.1))
            preScaledSpeedSetpoint *= -1;
    }
    return modm::Pair<float, float>(preScaledSpeedSetpoint, preScaledRotationSetpoint);
}

void SwerveWheel::refresh()
{
    drivePid.runControllerDerivateError(speedSetpointRPM - getDriveRPM(), 2.0f);
    driveMotor.setDesiredOutput(drivePid.getOutput());

    azimuthPid.runController(rotationSetpoint - getAngle(), getAngularVelocity(), 2.0f);
    azimuthMotor.setDesiredOutput(azimuthPid.getOutput());
}

float SwerveWheel::getDriveVelocity() const { return rpmToMps(driveMotor.getShaftRPM()); }

void SwerveWheel::setZeroRPM() { speedSetpointRPM = 0; }

float SwerveWheel::getDriveRPM() const { return driveMotor.getShaftRPM(); }

float SwerveWheel::getAngle() const
{
    return modm::toRadian(
        azimuthMotor.encoderToDegrees(azimuthMotor.getEncoderUnwrapped() - azimuthConfig.azimuthZeroOffset) *
        azimuthConfig.azimuthMotorGearing);
}

void SwerveWheel::initialize()
{
    driveMotor.initialize();
    azimuthMotor.initialize();
}

bool SwerveWheel::allMotorsOnline() const
{
    return driveMotor.isMotorOnline() && azimuthMotor.isMotorOnline();
}

float SwerveWheel::getAngularVelocity() const
{
    return 6.0f * static_cast<float>(azimuthMotor.getShaftRPM()) * (azimuthConfig.azimuthMotorGearing);
}

}  // namespace chassis

}  // namespace aruwsrc
