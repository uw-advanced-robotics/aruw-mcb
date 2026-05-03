#ifndef SERVO_SUBSYSTEM_HPP_
#define SERVO_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "aruwsrc/communication/mcb-lite/motor/virtual_servo.hpp"
#include "aruwsrc/communication/mcb-lite/mcb_lite.hpp"

namespace aruwsrc::engineer
{
class ServoSubsystem : public tap::control::Subsystem
{
public:
    ServoSubsystem(tap::Drivers* drivers, aruwsrc::communication::mcb_lite::MCBLite& mcbLite)
        : tap::control::Subsystem(drivers),
          servoOne(drivers, tap::gpio::Pwm::Pin::X, 1.0f, 0.0f, 0.01f, mcbLite.pwm),
          servoTwo(drivers, tap::gpio::Pwm::Pin::Buzzer, 1.0f, 0.0f, 0.01f, mcbLite.pwm), mcbLite(mcbLite) {}

    void refresh() override
    {
        servoOne.updateSendPwmRamp();
        servoTwo.updateSendPwmRamp();
    }

    void refreshSafeDisconnect() override {
        mcbLite.pwm.write(0.0f, servoOne.getPin());
        mcbLite.pwm.write(0.0f, servoTwo.getPin());
    } 

    aruwsrc::communication::mcb_lite::motor::VirtualServo servoOne;
    aruwsrc::communication::mcb_lite::motor::VirtualServo servoTwo;
    aruwsrc::communication::mcb_lite::MCBLite& mcbLite;

};
}
#endif