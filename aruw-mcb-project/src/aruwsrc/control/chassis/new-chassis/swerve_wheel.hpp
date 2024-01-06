
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

#include "wheel.hpp"
#include "modm/container/pair.hpp"
#include "modm/math/filter/pid.hpp"
using Motor = tap::motor::DjiMotor;
using SmoothPid = tap::algorithms::SmoothPid;
using SmoothPidConfig = tap::algorithms::SmoothPidConfig;

namespace aruwsrc
{
namespace chassis
{
class SwerveWheel : public Wheel {

public:
SwerveWheel(
        Motor& driveMotor,
        Motor& azimuthMotor,
        WheelConfig& config,
        SmoothPid drivePid,
        SmoothPid azimuthPid);
    float getAngularVelocity() const;

    float getAngle() const;

private:
    Motor& driveMotor;
    Motor& azimuthMotor;

    SmoothPid drivePid;
    SmoothPid azimuthPid;
    
    float rotationSetpoint, speedSetpointRPM;  // pid setpoint, in radians and rpm respectively

    float preScaledSpeedSetpoint{0}, preScaledRotationSetpoint{0}, newRawRotationSetpointRadians,
        newRotationSetpointRadians, moveVectorX, moveVectorY;

    // handles unwrapping desired rotation and reversing module (in radians, will always be a
    // multiple of PI)
    float rotationOffset{0};

};
}
}