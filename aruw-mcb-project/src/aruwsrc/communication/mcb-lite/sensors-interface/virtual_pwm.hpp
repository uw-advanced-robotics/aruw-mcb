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

#include "aruwsrc/communication/mcb-lite/message_types.hpp"

namespace aruwsrc::virtualMCB
{

// Struct of message being sent to MCBLite
struct PWMPinDutyMessage
{
    float WPinDuty;
    float XPinDuty;
    float YPinDuty;
    float ZPinDuty;
    float BuzzerPinDuty;
    float IMUHeaterPinDuty;
} modm_packed;

struct PWNTimerFrequencyMessage
{
    uint32_t timer8Frequency;
    uint32_t timer12Frequency;
    uint32_t timer3Frequency;
} modm_packed;

struct PWMTimerStartedMessage
{
    bool timer8Started;
    bool timer12Started;
    bool timer3Started;
} modm_packed;

class VirtualPwm : public tap::gpio::Pwm
{
    friend class SerialMCBLite;

public:
    VirtualPwm();

    // Please dont't call this, shouldn't be used
    void init() {}

    /**
     * Sets all configured timer channels to 0% duty cycle.
     */
    void writeAllZeros();

    /**
     * Sets the PWM duty for a specified pin.
     *
     * @param [in] duty the duty cycle to be set. If the duty is outside of the range
     *      of [0, 1] the duty is limited to within the range.
     * @param[in] pin the PWM pin to be set.
     */
    void write(float duty, Pwm::Pin pin);

    /**
     * Set the frequency of the timer, in Hz. Does nothing if frequency == 0
     */
    void setTimerFrequency(Timer timer, uint32_t frequency);

    void pause(Timer timer);

    void start(Timer timer);

private:
    float WPinDuty, XPinDuty, YPinDuty, ZPinDuty, BuzzerPinDuty, IMUHeaterPinDuty;

    uint32_t timer8Frequency, timer12Frequency, timer3Frequency;

    bool timer8Started, timer12Started, timer3Started;

    bool hasNewMessageData = false;

    tap::communication::serial::DJISerial::DJISerial::SerialMessage<sizeof(PWMPinDutyMessage)>
        pinDutyMessage;
    tap::communication::serial::DJISerial::DJISerial::SerialMessage<sizeof(
        PWNTimerFrequencyMessage)>
        timerFrequencyMessage;
    tap::communication::serial::DJISerial::DJISerial::SerialMessage<sizeof(PWMTimerStartedMessage)>
        timerStartedMessage;
};

}  // namespace aruwsrc::virtualMCB

#endif
