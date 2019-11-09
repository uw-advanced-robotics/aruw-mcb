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
        // aruwlib::motor::DjiMotor* motorLeft =
        //     reinterpret_cast<aruwlib::motor::DjiMotor*>(
        //     frictionWheelLeft.getPointer());
        // aruwlib::motor::DjiMotor* motorRight =
        //     reinterpret_cast<aruwlib::motor::DjiMotor*>(
        //     frictionWheelRight.getPointer());
        
        // modm::Pid<float>* veloPidLeft =
        //     reinterpret_cast<modm::Pid<float>*>(
        //     velocityPidLeftWheel.getPointer());

        // modm::Pid<float>* veloPidRight =
        //     reinterpret_cast<modm::Pid<float>*>(
        //     velocityPidRightWheel.getPointer());

        velocityPidLeftWheel->update(desiredRpm - frictionWheelLeft->getShaftRPM());
        velocityPidRightWheel->update(desiredRpm - frictionWheelRight->getShaftRPM());

        frictionWheelLeft->setDesiredOutput(velocityPidLeftWheel->getValue());
        frictionWheelRight->setDesiredOutput(velocityPidRightWheel->getValue());

        
        // frictionWheelLeft->setDesiredOutput(desiredRpm);
        // frictionWheelRight->setDesiredOutput(desiredRpm);
    }

}  // namespace control

}  // namespace aruwsrc
