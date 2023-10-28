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

#include "virtual_pwm.hpp"


namespace aruwsrc::virtualMCB
{
VirtualPwm::VirtualPwm() : Pwm(), pinDutyMessage(), timerFrequencyMessage(), timerStartedMessage()
{
    setDefaultTimerFrequencies();
    pinDutyMessage.messageType = PWM_PIN_DUTY_MESSAGE;
    timerStartedMessage.messageType = PWM_TIMER_STARTED_MESSAGE;
    timerFrequencyMessage.messageType = PWM_TIMER_FREQUENCY_MESSAGE;
}

void VirtualPwm::writeAllZeros()
{
    WPinDuty = 0;
    XPinDuty = 0;
    YPinDuty = 0;
    ZPinDuty = 0;
    BuzzerPinDuty = 0;
    IMUHeaterPinDuty = 0;

    memset(&pinDutyMessage.data, 0, sizeof(pinDutyMessage.data));

    hasNewMessageData = true;
    pinDutyMessage.setCRC16();
}

void VirtualPwm::write(float duty, Pwm::Pin pin)
{
    switch (pin)
    {
        case Pwm::Pin::W:
            WPinDuty = duty;
            break;
        case Pwm::Pin::X:
            XPinDuty = duty;
            break;
        case Pwm::Pin::Y:
            YPinDuty = duty;
            break;
        case Pwm::Pin::Z:
            ZPinDuty = duty;
            break;
        case Pwm::Pin::Buzzer:
            BuzzerPinDuty = duty;
            break;
        case Pwm::Pin::ImuHeater:
            IMUHeaterPinDuty = duty;
            break;
        default:
            break;
    }

    // pinDutyMessage.data[pin * sizeof(float)] = duty;
    memcpy(pinDutyMessage.data + (pin * sizeof(float)), &duty, sizeof(float));

    hasNewMessageData = true;
    pinDutyMessage.setCRC16();
}

void VirtualPwm::setTimerFrequency(Pwm::Timer timer, uint32_t frequency)
{
    switch (timer)
    {
        case Pwm::Timer::TIMER8:
            timer8Frequency = frequency;
            break;
        case Pwm::Timer::TIMER12:
            timer12Frequency = frequency;
            break;
        case Pwm::Timer::TIMER3:
            timer3Frequency = frequency;
            break;
        default:
            break;
    }
    // timerFrequencyMessage.data[timer * sizeof(uint32_t)] = frequency;
    memcpy(timerFrequencyMessage.data + (timer * sizeof(uint32_t)), &frequency, sizeof(uint32_t));

    hasNewMessageData = true;
    timerFrequencyMessage.setCRC16();
}

void VirtualPwm::pause(Pwm::Timer timer)
{
    switch (timer)
    {
        case Pwm::Timer::TIMER8:
            timer8Started = false;
            break;
        case Pwm::Timer::TIMER12:
            timer12Started = false;
            break;
        case Pwm::Timer::TIMER3:
            timer3Started = false;
            break;
        default:
            break;
    }
    timerStartedMessage.data[timer] = false;

    hasNewMessageData = true;
    timerStartedMessage.setCRC16();
}

void VirtualPwm::start(Pwm::Timer timer)
{
    switch (timer)
    {
        case Pwm::Timer::TIMER8:
            timer8Started = true;
            break;
        case Pwm::Timer::TIMER12:
            timer12Started = true;
            break;
        case Pwm::Timer::TIMER3:
            timer3Started = true;
            break;
        default:
            break;
    }
    timerStartedMessage.data[timer] = true;

    hasNewMessageData = true;
    timerStartedMessage.setCRC16();
}

}  // namespace aruwsrc::virtualMCB
