/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "agitator_fan_subsystem.hpp"

namespace aruwsrc::control::agitator
{
AgitatorFanSubsystem::AgitatorFanSubsystem(
    tap::Drivers* drivers,
    tap::gpio::Pwm::Pin pwmPin,
    tap::gpio::Pwm::Timer pwmTimer,
    uint32_t pwmFrequencyHz)
    : tap::control::Subsystem(drivers),
      pwmPin(pwmPin),
      pwmTimer(pwmTimer),
      pwmFrequencyHz(pwmFrequencyHz)
{
}

void AgitatorFanSubsystem::initialize()
{
    drivers->pwm.setTimerFrequency(pwmTimer, pwmFrequencyHz);
    setFanDuty(0.0f);
}

void AgitatorFanSubsystem::refreshSafeDisconnect() { setFanDuty(0.0f); }

void AgitatorFanSubsystem::setFanDuty(float duty) { drivers->pwm.write(duty, pwmPin); }

}  // namespace aruwsrc::control::agitator
