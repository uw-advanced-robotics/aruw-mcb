#include "src/control/example-subsystem.hpp"

namespace aruwsrc
{

namespace control
{
    ExampleSubsystem::ExampleSubsystem() :
        leftWheel(LEFT_MOTOR_ID, CAN_BUS_MOTORS),
        rightWheel(RIGHT_MOTOR_ID, CAN_BUS_MOTORS),
        velocityPidLeftWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        velocityPidRightWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
        desiredRpm(0)
    {}

    void ExampleSubsystem::setDesiredRpm(float desRpm)
    {
        desiredRpm = desRpm;
    }

    void ExampleSubsystem::refresh()
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

    void ExampleSubsystem::updateMotorRpmPid(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* motor,
        float desiredRpm
    ) {
        pid->update(desiredRpm - motor->getShaftRPM());
        motor->setDesiredOutput(pid->getValue());
    }
}  // namespace control

}  // namespace aruwsrc
