#include "engineer_wrist_subsystem.hpp"

namespace aruwsrc
{

namespace control
{
    EngineerWristSubsystem::EngineerWristSubsystem(uint16_t gearRatio) :
        leftMotor(LEFT_MOTOR_ID, CAN_BUS_MOTORS),
        rightMotor(RIGHT_MOTOR_ID, CAN_BUS_MOTORS),
        leftPositionPid(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        rightPositionPid(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        desiredWristAngle(0.0f),
        wristCalibrationAngle(0.0f),
        wristIsCalibrated(false),
        wristGearRatio(0.0f)
    {}

    void EngineerWristSubsystem::refresh()
    {
        wristRunPositionPid();
    }

    void EngineerWristSubsystem::setWristAngle(float newAngle)
    {

    }

    float EngineerWristSubsystem::getWristEncoderToPosition(void) const
    {

    }

    float EngineerWristSubsystem::getWristEncoderToPosition(void) const
    {

    }

    bool EngineerWristSubsystem::wristCalibrateHere(void)
    {
        if (!leftMotor.isMotorOnline() || !rightMotor.isMotorOnline())
        {
            return false;
        }
        //TODO encoder to angle calculation
        wristCalibrationAngle = 0;
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
        //TODO separate getWristEncoderToPosition to one for each motor
        leftPositionPid.update(desiredWristAngle - );
        rightPositionPid.update(desiredWristAngle - );
        leftMotor.setDesiredOutput(leftPositionPid.getValue());
        rightMotor.setDesiredOutput(rightPositionPid.getValue());
    }
}  // namespace control

}  // namespace aruwsrc