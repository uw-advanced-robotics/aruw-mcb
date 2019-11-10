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
        velocityPidLeftWheel->update(desiredRpm - frictionWheelLeft->getShaftRPM());
        velocityPidRightWheel->update(desiredRpm - frictionWheelRight->getShaftRPM());

        frictionWheelLeft->setDesiredOutput(velocityPidLeftWheel->getValue());
        frictionWheelRight->setDesiredOutput(velocityPidRightWheel->getValue());
    }
}  // namespace control

}  // namespace aruwsrc
