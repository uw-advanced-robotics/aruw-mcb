#include "src/control/example-subsystem.hpp"

namespace aruwsrc
{

namespace control
{
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
