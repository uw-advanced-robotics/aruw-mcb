#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

#include "modm/container/pair.hpp"
#include "modm/math/filter/pid.hpp"

#include "wheel.hpp"
using Motor = tap::motor::DjiMotor;
using SmoothPid = tap::algorithms::SmoothPid;
using SmoothPidConfig = tap::algorithms::SmoothPidConfig;

#include "swerve_wheel.hpp"
namespace aruwsrc
{
namespace chassis
{
SwerveWheel::SwerveWheel(
    Motor& driveMotor,
    Motor& azimuthMotor,
    WheelConfig& config,
    SmoothPid drivePid,
    SmoothPid azimuthPid)
    : Wheel(driveMotor, config),
      driveMotor(driveMotor),
      azimuthMotor(azimuthMotor),
      drivePid(drivePid),
      azimuthPid(azimuthPid)
{
    rotationSetpoint = 0;
    speedSetpointRPM = 0;
}

};  // namespace chassis
}  // namespace aruwsrc
