/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef VIRTUAL_SERVO_HPP_
#define VIRTUAL_SERVO_HPP_

#include "tap/communication/serial/dji_serial.hpp"
#include "tap/motor/servo.hpp"

#include "aruwsrc/communication/mcb-lite/message_types.hpp"
#include "aruwsrc/communication/mcb-lite/virtual_pwm.hpp"

using namespace tap::communication::serial;
namespace aruwsrc::communication::mcb_lite
{
class MCBLite;
}
namespace aruwsrc::communication::mcb_lite::motor
{
class VirtualServo : public tap::motor::Servo
{
public:
    VirtualServo(
        tap::Drivers* drivers,
        tap::gpio::Pwm::Pin pwmPin,
        float maximumPwm,
        float minimumPwm,
        float rampSpeed,
        aruwsrc::communication::mcb_lite::VirtualPWM& virtualPwm)
        : Servo(drivers, pwmPin, maximumPwm, minimumPwm, rampSpeed),
          virtualPwm(virtualPwm),
          pwmOutputRamp(0.0f),
          maxPwm(maximumPwm),
          minPwm(minimumPwm),
          rampSpeed(rampSpeed),
          pin(pwmPin)
    {
    }

    // theoretically no one should have a servo bob = virtualservo
    // so this function shadowing is fine hopefully? (its not virtual in tap)

    void setTargetPwm(float pwm)
    {
        pwmOutputRamp.setTarget(tap::algorithms::limitVal<float>(pwm, minPwm, maxPwm));
        prevTime = tap::arch::clock::getTimeMilliseconds();
    }
int bob = 0;
    void updateSendPwmRamp()
    {
        uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
        pwmOutputRamp.update(rampSpeed * (currTime - prevTime));
        prevTime = currTime;
        currentPwm = pwmOutputRamp.getValue();
        virtualPwm.write(currentPwm, pin);
        bob+=15;
    }

    float getPWM() const { return currentPwm; }

    bool isRampTargetMet() const { return pwmOutputRamp.isTargetReached(); }

    tap::gpio::Pwm::Pin getPin() const { return pin; }

private:
    aruwsrc::communication::mcb_lite::VirtualPWM& virtualPwm;

    tap::algorithms::Ramp pwmOutputRamp;
    uint32_t prevTime = 0;
    float maxPwm, minPwm, rampSpeed;
    tap::gpio::Pwm::Pin pin;
    float currentPwm = 0;
    bool isTargetReached = 0;
};
}  // namespace aruwsrc::communication::mcb_lite::motor

#endif  // VIRTUAL_SERVO_HPP_