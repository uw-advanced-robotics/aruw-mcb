#include "src/control/subsystem-example.hpp"

namespace aruwsrc
{

namespace control
{

    void InitDefaultCommand(void)
    {
        
    }

    void SubsystemExample::updatevelocityPidLeftWheelLoop(float desRpm)
    {
        velocityPidLeftWheel->update(desRpm - frictionWheelLeft->getShaftRPM());
        velocityPidRightWheel->update(desRpm - frictionWheelRight->getShaftRPM());
    }

}  // namespace control

}  // namespace aruwsrc
