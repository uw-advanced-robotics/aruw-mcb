#ifndef __DRONE_TURRET_SUBSYSTEM_HPP__
#define __DRONE_TURRET_SUBSYSTEM_HPP__

#include "src/aruwlib/communication/gpio/pwm.hpp"
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace drone
{

class DroneTurretSubsystem : public Subsystem
{
private:

    static const aruwlib::gpio::Pwm::Pin leftFrictionWheelPin = aruwlib::gpio::Pwm::Pin::X;
    static const aruwlib::gpio::Pwm::Pin rightFrictionWheelPin = aruwlib::gpio::Pwm::Pin::Y;

    aruwlib::gpio::Pwm leftFrictionWheel; // View from top
    aruwlib::gpio::Pwm rightFrictionWheel; // View from top
    
    bool initialized;
    float currentFrictionWheelPWMDuty;
    friend class InitFrictionWheelCommand;

    void setRawFrictionWheelOutput(float duty);

public:
    static constexpr float MIN_PWM_DUTY = 0.1f;
    static constexpr float MAX_PWM_DUTY = 0.2f;

    DroneTurretSubsystem() :
            leftFrictionWheel(),
            rightFrictionWheel(),
            initialized(false),
            currentFrictionWheelPWMDuty(0.1f)
            {};
    
    void refresh() override;
    void setFrictionWheelOutput(float percentage);
    void stopFrictionWheel();
};

}

}

#endif