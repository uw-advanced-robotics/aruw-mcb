#include "wrist_subsystem.hpp"

namespace aruwsrc
{

namespace engineer
{
        float test;
    WristSubsystem::WristSubsystem() :
        leftMotor(LEFT_MOTOR_ID, CAN_BUS_MOTORS, true),
        rightMotor(RIGHT_MOTOR_ID, CAN_BUS_MOTORS, false),
        leftPositionPid(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        rightPositionPid(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        desiredWristAngleLeft(0.0f),
        desiredWristAngleRight(0.0f),
        wristCalibratedAngleLeft(0.0f),
        wristCalibratedAngleRight(0.0f),
        wristIsCalibrated(false),
        wristIsIn(true)
    {}

    void WristSubsystem::refresh()
    {
        if (wristIsCalibrated)
        {
            wristRunPositionPid();
            test = leftMotor.encStore.getEncoderUnwrapped();
        }
        else
        {
            wristCalibrateHere();
        }
    }

    void WristSubsystem::setWristAngleLeft(float newAngle)
    {
        desiredWristAngleLeft = newAngle;
    }

    void WristSubsystem::setWristAngleRight(float newAngle)
    {
        desiredWristAngleRight = newAngle;
    }

    float WristSubsystem::getWristAngleLeft(void) const
    {
        if (!wristIsCalibrated)
        {
            return 0.0f;
        }
        return getUncalibratedWristAngleLeft() - wristCalibratedAngleLeft;
    }

    float WristSubsystem::getWristAngleRight(void) const
    {
        if (!wristIsCalibrated)
        {
            return 0.0f;
        }
        return getUncalibratedWristAngleRight() - wristCalibratedAngleRight;
    }

    float WristSubsystem::getWristDesiredAngleLeft(void) const
    {
        return desiredWristAngleLeft;
    }

    float WristSubsystem::getWristDesiredAngleRight(void) const
    {
        return desiredWristAngleRight;
    }

    bool WristSubsystem::wristCalibrateHere(void)
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

    void WristSubsystem::wristToggleState()
    {
        wristIsIn = !wristIsIn;
    }

    bool WristSubsystem::isIn()
    {
        return wristIsIn;
    }

    void WristSubsystem::wristRunPositionPid(void)
    {
        if (!wristIsCalibrated)
        {
            leftPositionPid.reset();
            rightPositionPid.reset();
            return;
        }
        leftPositionPid.update(desiredWristAngleLeft - getWristAngleLeft());
        rightPositionPid.update(desiredWristAngleRight - getWristAngleRight());
        leftMotor.setDesiredOutput(leftPositionPid.getValue());
        rightMotor.setDesiredOutput(rightPositionPid.getValue());
    }

    // position = 2 * PI / encoder resolution * unwrapped encoder value / gear ratio
    float WristSubsystem::getUncalibratedWristAngleLeft(void) const
    {
        return (2.0f * aruwlib::algorithms::PI / static_cast<float>(ENC_RESOLUTION)) *
            leftMotor.encStore.getEncoderUnwrapped() / WRIST_GEAR_RATIO;
    }

    float WristSubsystem::getUncalibratedWristAngleRight(void) const
    {
        return (2.0f * aruwlib::algorithms::PI / static_cast<float>(ENC_RESOLUTION)) *
            rightMotor.encStore.getEncoderUnwrapped() / WRIST_GEAR_RATIO;
    }
}  // namespace engineer

}  // namespace aruwsrc