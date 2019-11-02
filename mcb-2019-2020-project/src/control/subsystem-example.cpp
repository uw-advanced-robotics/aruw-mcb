#include "src/control/subsystem-example.hpp"

namespace aruwsrc
{

namespace control // should there be some sort of update function for a subsystem. other alternative is a default command.
{ // should a subsystem store a default command?

    void InitDefaultCommand(void)
    {

    }

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
