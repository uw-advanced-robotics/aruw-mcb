#include "src/control/example-subsystem.hpp"

namespace aruwsrc
{

namespace control
{
    ExampleSubsystem::ExampleSubsystem(
        float p,
        float i,
        float d,
        float maxErrorSum,
        float maxOut,
        aruwlib::motor::MotorId leftMotorId,
        aruwlib::motor::MotorId rightMotorId
    ) : desiredRpm(0) {
        leftWheel = new aruwlib::motor::DjiMotor(leftMotorId,
            aruwlib::can::CanBus::CAN_BUS1);
        rightWheel = new aruwlib::motor::DjiMotor(rightMotorId,
            aruwlib::can::CanBus::CAN_BUS1);
        velocityPidLeftWheel = modm::SmartPointer(
            new modm::Pid<float>(p, i, d, maxErrorSum, maxOut));
        velocityPidRightWheel = modm::SmartPointer(
            new modm::Pid<float>(p, i, d, maxErrorSum, maxOut));
    }

    void ExampleSubsystem::setDesiredRpm(float desRpm)
    {
        desiredRpm = desRpm;
    }

    void ExampleSubsystem::refresh()
    {
        updateMotorRpmPid(
            getPidPointer(velocityPidLeftWheel),
            leftWheel, desiredRpm
        );
        updateMotorRpmPid(
            getPidPointer(velocityPidRightWheel),
            rightWheel, desiredRpm
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

    modm::Pid<float>* ExampleSubsystem::getPidPointer(modm::SmartPointer smrtPtr)
    {
        return reinterpret_cast<modm::Pid<float>*>(smrtPtr.getPointer());
    }
}  // namespace control

}  // namespace aruwsrc
