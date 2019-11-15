#include "src/control/subsystem-example.hpp"

namespace aruwsrc
{

namespace control
{
    SubsystemExample::SubsystemExample(
        float p,
        float i,
        float d,
        float maxErrorSum,
        float maxOut,
        aruwlib::motor::MotorId leftMotorId,
        aruwlib::motor::MotorId rightMotorId
    ) {
        m1 = new aruwlib::motor::DjiMotor(leftMotorId,
            aruwlib::can::CanBus::CAN_BUS1);
        m2 = new aruwlib::motor::DjiMotor(rightMotorId,
            aruwlib::can::CanBus::CAN_BUS1);
        velocityPidLeftWheel = modm::SmartPointer(
            new modm::Pid<float>(p, i, d, maxErrorSum, maxOut));
        velocityPidRightWheel = modm::SmartPointer(
            new modm::Pid<float>(p, i, d, maxErrorSum, maxOut));
    }

    void SubsystemExample::setDesiredRpm(float desRpm)
    {
        desiredRpm = desRpm;
    }

    void SubsystemExample::refresh()
    {
        updateMotorRpmPid(
            getPidPointer(velocityPidLeftWheel),
            m1, desiredRpm
        );
        updateMotorRpmPid(
            getPidPointer(velocityPidRightWheel),
            m2, desiredRpm
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
}  // namespace control

}  // namespace aruwsrc
