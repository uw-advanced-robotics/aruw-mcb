
#include "drone_turret_subsystem.hpp"

using namespace aruwlib;

using namespace aruwlib::algorithms;


namespace aruwsrc
{

namespace drone
{

void DroneTurretSubsystem::setFrictionWheelOutput(float percentage) {
    if (!initialized)
    {
        stopFrictionWheel();
    }
    else {
        currentFrictionWheelPWMDuty = 
                mapValLimited<float>(percentage, 0.0f, 1.0f, MIN_PWM_DUTY, MAX_PWM_DUTY);
        setRawFrictionWheelOutput(currentFrictionWheelPWMDuty);
    }
}

void DroneTurretSubsystem::setRawFrictionWheelOutput(float duty) {
    leftFrictionWheel.Write(duty, leftFrictionWheelPin);
    rightFrictionWheel.Write(duty, rightFrictionWheelPin);
    currentFrictionWheelPWMDuty = duty;
}

void DroneTurretSubsystem::stopFrictionWheel() {
    setRawFrictionWheelOutput(MIN_PWM_DUTY);
}

void DroneTurretSubsystem::refresh() {
    
}

}

}