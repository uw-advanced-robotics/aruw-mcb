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


using namespace tap::communication::serial;
namespace aruwsrc::communication::mcb_lite {
class MCBLite;
}
namespace aruwsrc::communication::mcb_lite::motor
{

class VirtualServo : public tap::motor::Servo
{
    friend class aruwsrc::communication::mcb_lite::MCBLite;
    friend class VirtualServoRxHandler;

public:
    VirtualServo(
    tap::Drivers *drivers,
    tap::gpio::Pwm::Pin pwmPin,
    float maximumPwm,
    float minimumPwm,
    float pwmRampSpeed,
    aruwsrc::communication::mcb_lite::MCBLite* mcbLite,
    bool isServoOne)
    : Servo(drivers, pwmPin, maximumPwm, minimumPwm, pwmRampSpeed), pin(pwmPin), minPwm(minimumPwm), maxPwm(maximumPwm), rampSpeed(pwmRampSpeed), mcbLite(mcbLite) {
        
        mcbLite->servoRxHandler.attachReceiveHandler(this, isServoOne);

        targetMessage.messageType = MessageTypes::SERVO_TARGET_MESSAGE;
        rampMessage.messageType = MessageTypes::SERVO_RAMP_MESSAGE;
        ServoRampMessage rampData;
        rampData.pin = pin;
        rampData.rampSpeed = pwmRampSpeed;
        memcpy(rampMessage.data, &rampData, sizeof(ServoRampMessage));
        rampMessage.setCRC16();
        hasNewRamp = true;

    };

    void setTargetPwm(float pwm)
{
    float targetPwm = tap::algorithms::limitVal<float>(pwm, minPwm, maxPwm);
    updateMessages(targetPwm);
    hasNewTarget = true;

}

float getPWM() const { return currentPwm; }  

bool isRampTargetMet() const { return isTargetReached; }

void attachSelfToRxHandler() {
    mcbLite->servoRxHandler.attachReceiveHandler(&motorOne);
    mcbLite->servoRxHandler.attachReceiveHandler(&motorTwo);
}

private: 
    void processServoUARTMessage( float currentPwm, bool isRampTargetMet) {
        this->currentPwm = currentPwm;
        this->isTargetReached = isRampTargetMet;
    }

    void updateMessages(float pwm) {
        ServoTargetMessage targetData;
        targetData.pin = pin;
        targetData.target = pwm;
        memcpy(targetMessage.data, &targetData, sizeof(ServoTargetMessage));
        targetMessage.setCRC16();
    }



    float minPwm, maxPwm, rampSpeed;
    tap::gpio::Pwm::Pin pin;
    aruwsrc::communication::mcb_lite::MCBLite* mcbLite;
    float currentPwm = 0;
    bool hasNewTarget = 0; 
    bool hasNewRamp = 0;   
    bool isTargetReached = 0;
    DJISerial::SerialMessage<sizeof(ServoTargetMessage)> targetMessage;
    DJISerial::SerialMessage<sizeof(ServoRampMessage)> rampMessage;
};
}  // namespace aruwsrc::communication::mcb_lite::motor

#endif  // VIRTUAL_SERVO_HPP_
