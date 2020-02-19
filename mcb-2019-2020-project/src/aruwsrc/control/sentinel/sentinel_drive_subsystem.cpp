#include "sentinel_drive_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"

namespace aruwsrc
{

namespace control
{
    const aruwlib::motor::MotorId SentinelDriveSubsystem::LEFT_MOTOR_ID = aruwlib::motor::MOTOR2;
    const aruwlib::motor::MotorId SentinelDriveSubsystem::RIGHT_MOTOR_ID = aruwlib::motor::MOTOR3;

    void SentinelDriveSubsystem::setDesiredRpm(float desRpm)
    {
        desiredRpm = desRpm;
    }

    void SentinelDriveSubsystem::refresh()
    {
        updateMotorRpmPid(
            &velocityPidLeftWheel,
            &leftWheel, desiredRpm
        );
        updateMotorRpmPid(
            &velocityPidRightWheel,
            &rightWheel, desiredRpm
        );
    }

    void SentinelDriveSubsystem::updateMotorRpmPid(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* motor,
        float desiredRpm
    ) {
        pid->update(desiredRpm - motor->getShaftRPM());
        motor->setDesiredOutput(pid->getValue());
    }

    // Returns the absolute position of the sentinel on the rail with respect to the first rail
    // end that was hit
    float SentinelDriveSubsystem::absolutePosition ()
    {
        return ((distanceFromEncoder(&leftWheel) - leftZeroRailOffset) +
            (distanceFromEncoder(&rightWheel) - rightZeroRailOffset)) / 2;
    }

    // Call when rail is hit to set the offset or the
    // Sets the offset field value every time a rails is hit
    // Only sets the offset when the rail end that was first hit is hit again, or a rail end
    // is hit for the first time
    void SentinelDriveSubsystem::resetOffsetFromLimitSwitch ()
    {
        if (leftLimitSwitch::read()) {  // DigitalPin where LimitterSwitch is placed
            leftZeroRailOffset = distanceFromEncoder(&leftWheel);
            rightZeroRailOffset = distanceFromEncoder(&rightWheel);
        } else if (rightLimitSwitch::read()) {
            leftZeroRailOffset = RAIL_LENGTH - distanceFromEncoder(&leftWheel);
            rightZeroRailOffset = RAIL_LENGTH - distanceFromEncoder(&rightWheel);
        }
    }

    // Returns the distance covered by the sentinel wheel on the rail
    // with respect to the encoders
    // Equation used: Arc Length = Angle * numberOfRotations * radius
    // Here we get the radius from the getEncoderUnwrapped function
    float SentinelDriveSubsystem::distanceFromEncoder(aruwlib::motor::DjiMotor* motor){
        float unwrappedAngle = motor->encStore.getEncoderUnwrapped();
        float numberOfRotations = unwrappedAngle / (aruwlib::motor::DjiMotor::ENC_RESOLUTION + 1);
        return numberOfRotations * 2.0f * aruwlib::algorithms::PI * WHEEL_RADIUS / GEAR_RATIO;
    }

}  // namespace control

}  // namespace aruwsrc
