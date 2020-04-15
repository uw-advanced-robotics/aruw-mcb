#ifndef __DRONE_TURRET_SUBSYSTEM_HPP__
#define __DRONE_TURRET_SUBSYSTEM_HPP__

#include "src/aruwlib/communication/gpio/pwm.hpp"
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/algorithms/ramp.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace drone
{

class PWMFrictionWheelSubsystem : public Subsystem
{
 public:
    static constexpr float MIN_PWM_DUTY = 0.1f;
    static constexpr float MAX_PWM_DUTY = 0.2f;

 private:
    static const aruwlib::gpio::Pwm::Pin leftFrictionWheelPin = aruwlib::gpio::Pwm::Pin::X;
    static const aruwlib::gpio::Pwm::Pin rightFrictionWheelPin = aruwlib::gpio::Pwm::Pin::Y;

    aruwlib::gpio::Pwm leftFrictionWheel;  // View from top
    aruwlib::gpio::Pwm rightFrictionWheel;  // View from top

    bool initialized;
    float currentFrictionWheelPWMDuty;

    static constexpr uint32_t RAMP_TIME_MS = 250;
    static constexpr float RAMP_RATE = (MAX_PWM_DUTY - MIN_PWM_DUTY) / RAMP_TIME_MS;
    uint32_t lastRampTime;
    aruwlib::algorithms::Ramp throttleRamp;

    void setRawFrictionWheelOutput(float duty);

    friend class InitPWMFrictionWheelCommand;

 public:
    PWMFrictionWheelSubsystem() :
            leftFrictionWheel(),
            rightFrictionWheel(),
            initialized(false),
            currentFrictionWheelPWMDuty(0.1f),
            lastRampTime(0),
            throttleRamp(0.1f)
            {};

    void refresh() override;
    void setFrictionWheelOutput(float percentage);
    void stopFrictionWheel();
    bool isInitialized();
};

}  // namespace drone

}  // namespace aruwsrc

#endif
