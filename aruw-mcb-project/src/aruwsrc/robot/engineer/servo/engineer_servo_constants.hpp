/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ENGINEER_SERVO_CONSTANTS_HPP_
#define ENGINEER_SERVO_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/gpio/digital.hpp"
#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::engineer::servo
{
// figure this out (which is acc yaw & pitch; what the max & min angles r etc.)
static constexpr tap::gpio::Pwm::Pin YAW_PIN = tap::gpio::Pwm::Pin::X;
static constexpr tap::gpio::Pwm::Pin PITCH_PIN = tap::gpio::Pwm::Pin::Buzzer;

static constexpr float YAW_MIN_PWM = 0.0f;
static constexpr float YAW_MAX_PWM = 1.0f;
static constexpr float YAW_MIN_ANGLE = -M_PI_2;  // im assuming its not 360
static constexpr float YAW_MAX_ANGLE = M_PI_2;

static constexpr float PITCH_MIN_PWM = 0.0f;
static constexpr float PITCH_MAX_PWM = 1.0f;
static constexpr float PITCH_MIN_ANGLE = -M_PI_2;  // same here
static constexpr float PITCH_MAX_ANGLE = M_PI_2;

static constexpr float RAMP_SPEED = 0.01f;

}  // namespace aruwsrc::engineer::servo
#endif