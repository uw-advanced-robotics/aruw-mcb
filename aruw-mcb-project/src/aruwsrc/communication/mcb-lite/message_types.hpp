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

#ifndef MESSAGE_TYPES_HPP_
#define MESSAGE_TYPES_HPP_

#include "stdint.h"

namespace aruwsrc::virtualMCB
{
enum MessageTypes : uint8_t
{
    CANBUS1_MESSAGE = 0,
    CANBUS2_MESSAGE = 1,
    IMU_MESSAGE = 2,
    CURRENT_SENSOR_MESSAGE = 3,
    CALIBRATE_IMU_MESSAGE = 4,
    DIGITAL_OUTPUT_MESSAGE = 5,
    DIGITAL_PIN_MODE_MESSAGE = 6,
    DIGITAL_PID_READ_MESSAGE = 7,
    ANALOG_PIN_READ_MESSAGE = 8,
    PWM_PIN_DUTY_MESSAGE = 9,
    PWM_TIMER_MESSAGE = 10
};
}  // namespace aruwsrc::virtualMCB

#endif
