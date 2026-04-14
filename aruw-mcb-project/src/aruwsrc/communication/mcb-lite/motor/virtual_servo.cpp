// /*
//  * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
//  *
//  * This file is part of aruw-mcb.
//  *
//  * aruw-mcb is free software: you can redistribute it and/or modify
//  * it under the terms of the GNU General Public License as published by
//  * the Free Software Foundation, either version 3 of the License, or
//  * (at your option) any later version.
//  *
//  * aruw-mcb is distributed in the hope that it will be useful,
//  * but WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  * GNU General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License
//  * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
//  */

// #include "virtual_servo.hpp"

// #include "tap/communication/serial/dji_serial.hpp"
// #include "tap/motor/servo.hpp"

// #include "aruwsrc/communication/mcb-lite/mcb_lite.hpp"
// #include "aruwsrc/communication/mcb-lite/message_types.hpp"

// using namespace tap::communication::serial;

// namespace aruwsrc::communication::mcb_lite::motor
// {
// VirtualServo::VirtualServo(
//     tap::Drivers* drivers,
//     tap::gpio::Pwm::Pin pwmPin,
//     float minimumPwm,
//     float maximumPwm,
//     float pwmRampSpeed,
//     aruwsrc::communication::mcb_lite::VirtualPWM* virtualPwm)
//     : Servo(drivers, pwmPin, maximumPwm, minimumPwm, pwmRampSpeed),
//       virtualPwm(virtualPwm),
//       pin(pwmPin),
//       minPwm(minimumPwm),
//       maxPwm(maximumPwm),
//       rampSpeed(pwmRampSpeed)
//       //mcbLite(mcbLite)
// {
//     // mcbLite->servoRxHandler.attachReceiveHandler(this);

//     // targetMessage.messageType = MessageTypes::SERVO_TARGET_MESSAGE;
//     // rampMessage.messageType = MessageTypes::SERVO_RAMP_MESSAGE;
//     // ServoRampMessage rampData;
//     // rampData.pin = pin;
//     // rampData.maxPwm = maxPwm;
//     // rampData.minPwm = minPwm;
//     // rampData.rampSpeed = pwmRampSpeed;
//     // memcpy(rampMessage.data, &rampData, sizeof(ServoRampMessage));
//     // rampMessage.setCRC16();
//     // hasNewRamp = true;
// };

// void VirtualServo::setTargetPwm(float pwm)
// {
//     updateMessages(pwm);
//     hasNewTarget = true;
// }

// float VirtualServo::getPWM() const { return currentPwm; }

// tap::gpio::Pwm::Pin VirtualServo::getPin() const { return pin; }

// bool VirtualServo::isRampTargetMet() const { return isTargetReached; }

// void VirtualServo::processServoUARTMessage(float currentPwm, bool isRampTargetMet)
// {
//     this->currentPwm = currentPwm;
//     this->isTargetReached = isRampTargetMet;
// }

// // void VirtualServo::updateMessages(float pwm)
// // {
// //     ServoTargetMessage targetData;
// //     targetData.pin = pin;
// //     targetData.pwm = pwm;
// //     memcpy(targetMessage.data, &targetData, sizeof(ServoTargetMessage));
// //     targetMessage.setCRC16();
// // }

// }  // namespace aruwsrc::communication::mcb_lite::motor
//    // namespace aruwsrc::communication::mcb_lite::motor
