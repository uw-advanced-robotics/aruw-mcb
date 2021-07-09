#include "EngineerWristSubsystem.hpp"

#include "aruwlib/algorithms/math_user_utils.hpp"

using aruwlib::algorithms::limitVal;

namespace aruwsrc
{
namespace engineer
{
EngineerWristSubsystem::EngineerWristSubsystem(aruwlib::Drivers *drivers)
    : aruwlib::control::Subsystem(drivers),
      leftMotor(drivers, LEFT_MOTOR_ID, CAN_BUS_MOTORS, true, "engineer wrist left"),
      rightMotor(drivers, RIGHT_MOTOR_ID, CAN_BUS_MOTORS, false, "engineer wrist right"),
      leftPositionPid(
          PID_P,
          PID_I,
          PID_D,
          PID_MAX_ERROR_SUM,
          PID_MAX_OUTPUT,
          1.0f,
          0.0f,
          1.0f,
          0.0f),
      rightPositionPid(
          PID_P,
          PID_I,
          PID_D,
          PID_MAX_ERROR_SUM,
          PID_MAX_OUTPUT,
          1.0f,
          0.0f,
          1.0f,
          0.0f),
      desiredWristAngle(0.0f),
      wristCalibratedAngleLeft(0.0f),
      wristCalibratedAngleRight(0.0f),
      wristIsCalibrated(false)
{
}

void EngineerWristSubsystem::refresh()
{
    if (!wristIsCalibrated)
    {
        wristCalibrateHere();
    }
    else if (!leftMotor.isMotorOnline() || !rightMotor.isMotorOnline())
    {
        wristIsCalibrated = false;
    }
    else
    {
        wristRunPositionPid();
    }
}

void EngineerWristSubsystem::setWristAngle(float newAngle)
{
    desiredWristAngle = limitVal<float>(newAngle, MIN_WRIST_ANGLE, MAX_WRIST_ANGLE);
}

bool EngineerWristSubsystem::wristCalibrateHere()
{
    if (!leftMotor.isMotorOnline() || !rightMotor.isMotorOnline())
    {
        return false;
    }
    wristCalibratedAngleLeft = getUncalibratedWristAngleLeft();
    wristCalibratedAngleRight = getUncalibratedWristAngleRight();
    wristIsCalibrated = true;
    return true;
}

void EngineerWristSubsystem::wristRunPositionPid()
{
    if (!wristIsCalibrated)
    {
        leftPositionPid.reset();
        rightPositionPid.reset();
        return;
    }

    leftPositionPid.runControllerDerivateError(
        desiredWristAngle - getUncalibratedWristAngleLeft() - wristCalibratedAngleLeft,
        leftMotor.getShaftRPM());
    rightPositionPid.runControllerDerivateError(
        desiredWristAngle - getUncalibratedWristAngleRight() - wristCalibratedAngleRight,
        rightMotor.getShaftRPM());

    leftMotor.setDesiredOutput(leftPositionPid.getOutput());
    rightMotor.setDesiredOutput(rightPositionPid.getOutput());
}
}  // namespace engineer
}  // namespace aruwsrc
