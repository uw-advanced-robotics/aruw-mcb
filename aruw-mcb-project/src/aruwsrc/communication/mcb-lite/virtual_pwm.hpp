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

#ifndef VIRTUAL_PWM_HPP_
#define VIRTUAL_PWM_HPP_

#include "tap/communication/gpio/pwm.hpp"
#include "tap/communication/serial/dji_serial.hpp"

#include "message_types.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::virtualMCB
{
class VirtualPWM : public tap::gpio::Pwm
{
    friend class MCBLite;

public:
    VirtualPWM()
    {
        pinDutyMessage.messageType = MessageTypes::PWM_PIN_DUTY_MESSAGE;
        pwmTimerFrequencyMessage.messageType = MessageTypes::PWM_TIMER_FREQUENCY_MESSAGE;
        pwmTimerStartMessage.messageType = MessageTypes::PWM_TIMER_STARTED_MESSAGE;
    }

    void writeAllZeros() { memset(pinDuty, 0, sizeof(pinDuty)); }

    void setTimerFrequency(Timer timer, uint32_t frequency)
    {
        timerFrequency[timer] = frequency;
        updateMessages();
    }

    void start(Timer timer)
    {
        timerStarted[timer] = true;
        updateMessages();
    }

    void pause(Timer timer)
    {
        timerStarted[timer] = false;
        updateMessages();
    }

private:
    void updateMessages();

    float pinDuty[6];
    uint32_t timerFrequency[3];
    bool timerStarted[3];

    DJISerial::DJISerial::SerialMessage<sizeof(PWMPinDutyMessage)> pinDutyMessage;
    DJISerial::DJISerial::SerialMessage<sizeof(PWNTimerFrequencyMessage)> pwmTimerFrequencyMessage;
    DJISerial::DJISerial::SerialMessage<sizeof(PWMTimerStartedMessage)> pwmTimerStartMessage;

    bool hasNewData = false;
};
}  // namespace aruwsrc::virtualMCB

#endif  // VIRTUAL_PWM_HPP_
