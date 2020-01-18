#include "engineer_wrist_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

namespace aruwsrc
{

namespace control
{
    EngineerWristSubsystem::EngineerWristSubsystem() :
        // may need to flip which motor is inverted
        leftMotor(LEFT_MOTOR_ID, CAN_BUS_MOTORS, true),
        rightMotor(RIGHT_MOTOR_ID, CAN_BUS_MOTORS, false),
        leftPositionPid(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        rightPositionPid(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        desiredWristAngle(0.0f),
        wristCalibrationAngle(0.0f),
        wristIsCalibrated(false)
    {}

    void EngineerWristSubsystem::refresh()
    {
        wristRunPositionPid();
    }

    void EngineerWristSubsystem::setWristAngle(float newAngle)
    {
        desiredWristAngle = newAngle;
    }

    float EngineerWristSubsystem::getWristAngle(void) const
    {
        if (!wristIsCalibrated)
        {
            return 0.0f;
        }
        // TODO: Base the angle off of one motor or keep track of each individually???
        return getUncalibratedWristLeftAngle() - wristCalibrationAngle;
    }

    float EngineerWristSubsystem::getWristDesiredAngle(void) const
    {
        return desiredWristAngle;
    }

    bool EngineerWristSubsystem::wristCalibrateHere(void)
    {
        if (!leftMotor.isMotorOnline() || !rightMotor.isMotorOnline())
        {
            return false;
        }
        // TODO: Base the angle off of one motor or keep track of each individually???
        wristCalibrationAngle = getUncalibratedWristLeftAngle();
        wristIsCalibrated = true;
        return true;
    }

    void EngineerWristSubsystem::wristRunPositionPid(void)
    {
        if (!wristIsCalibrated)
        {
            leftPositionPid.reset();
            rightPositionPid.reset();
            return;
        }
        leftPositionPid.update(desiredWristAngle - getWristAngle());
        rightPositionPid.update(desiredWristAngle - getWristAngle());
        leftMotor.setDesiredOutput(leftPositionPid.getValue());
        rightMotor.setDesiredOutput(rightPositionPid.getValue());
    }

    // position = 2 * PI / encoder resolution * unwrapped encoder value / gear ratio
    float EngineerWristSubsystem::getUncalibratedWristLeftAngle(void) const
    {
        return (2.0f * aruwlib::algorithms::PI / static_cast<float>(ENC_RESOLUTION)) *
            leftMotor.encStore.getEncoderUnwrapped() / WRIST_GEAR_RATIO;
    }

    float EngineerWristSubsystem::getUncalibratedWristRightAngle(void) const
    {
        return (2.0f * aruwlib::algorithms::PI / static_cast<float>(ENC_RESOLUTION)) *
            rightMotor.encStore.getEncoderUnwrapped() / WRIST_GEAR_RATIO;
    }
}  // namespace control

}  // namespace aruwsrc