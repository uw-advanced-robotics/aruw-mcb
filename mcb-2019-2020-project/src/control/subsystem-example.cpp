#include "src/control/subsystem-example.hpp"

namespace aruwsrc
{

namespace control
{
    void SubsystemExample::setDesiredRpm(float desRpm)
    {
        desiredRpm = desRpm;
    }

    void SubsystemExample::refresh()
    {        
        updateMotorRpmPid(
            getPidPointer(velocityPidLeftWheel),
            getMotorPointer(frictionWheelLeft),
            desiredRpm
        );
        updateMotorRpmPid(
            getPidPointer(velocityPidRightWheel),
            getMotorPointer(frictionWheelRight),
            desiredRpm
        );
    }

    void SubsystemExample::updateMotorRpmPid(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* motor,
        float desiredRpm
    ) {
        pid->update(desiredRpm - motor->getShaftRPM());
        motor->setDesiredOutput(pid->getValue());
    }

    modm::Pid<float>* SubsystemExample::getPidPointer(modm::SmartPointer smrtPtr)
    {
        return reinterpret_cast<modm::Pid<float>*>(smrtPtr.getPointer());
    }

    aruwlib::motor::DjiMotor* SubsystemExample::getMotorPointer(modm::SmartPointer smrtPtr)
    {
        return reinterpret_cast<aruwlib::motor::DjiMotor*>(smrtPtr.getPointer());
    }
}  // namespace control

}  // namespace aruwsrc
