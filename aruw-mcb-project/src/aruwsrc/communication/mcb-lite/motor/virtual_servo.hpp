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

#include "tap/motor/servo.hpp"
#include "tap/communication/serial/dji_serial.hpp"

#include "aruwsrc/communication/mcb-lite/mcb_lite.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::communication::mcb_lite::motor
{
class VirtualServo : public tap::motor::Servo
{
    friend class MCBLite;
    friend class VirtualServoRxHandler;

public:
    VirtualServo(
    tap::Drivers *drivers,
    tap::gpio::Pwm::Pin pwmPin,
    float maximumPwm,
    float minimumPwm,
    float pwmRampSpeed,
    aruwsrc::communication::mcb_lite::MCBLite* mcbLite)
    : Servo(drivers, pwmPin, maximumPwm, minimumPwm, pwmRampSpeed), minPwm(minimumPwm), maxPwm(maximumPwm), rampSpeed(pwmRampSpeed), mcbLite(mcbLite) {};

    void setTargetPwm(float pwm)
{
    float targetPwm = tap::algorithms::limitVal<float>(pwm, minPwm, maxPwm);
    // send servo message
}

void updateSendPwmRamp() {} // send message

float getPWM() const {return currentPwm; } // need to read message 

bool isRampTargetMet() const { return isTargetReached; } // need to read message 

private: 
    void processServoUARTMessage( float currentPwm, bool isRampTargetMet) {
        this->currentPwm = currentPwm;
        this->isTargetReached = isRampTargetMet;
    }

    float minPwm, maxPwm, rampSpeed;
    tap::gpio::Pwm::Pin pin;
    aruwsrc::communication::mcb_lite::MCBLite* mcbLite;
    float currentPwm = 0;
    bool hasNewTarget, updatePwmRamp = 0;
    bool isTargetReached = 0;
};
}  // namespace aruwsrc::communication::mcb_lite::motor

#endif  // VIRTUAL_SERVO_HPP_
