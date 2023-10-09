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

VirtualPwm::VirtualPwm() : pinDutyMessage(), timerStartedMessage(), timerFrequencyMessage() {
	pinDutyMessage.messageType = PWM_PIN_DUTY_MESSAGE;
	timerStartedMessage.messageType = PWM_TIMER_STARTED_MESSAGE;
	timerFrequencyMessage.messageType = PWM_TIMER_FREQUENCY_MESSAGE;
}

void VirtualPwm::writeAllZeros(){
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



}  // namespace aruwsrc::virtualMCB
