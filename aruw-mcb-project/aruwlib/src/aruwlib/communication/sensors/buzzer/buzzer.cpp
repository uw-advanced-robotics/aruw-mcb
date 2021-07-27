/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruwlib.
 *
 * aruwlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruwlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruwlib.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "buzzer.hpp"

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/communication/gpio/pwm.hpp"

#include "modm/architecture/interface/delay.hpp"

namespace aruwlib::buzzer
{
void playNote(gpio::Pwm *pwmController, uint32_t frequency)
{
    if (frequency == 0)
    {
        silenceBuzzer(pwmController);
    }
    else
    {
        pwmController->write(0.5f, gpio::Pwm::BUZZER);
        pwmController->setTimerFrequency(gpio::Pwm::TIMER_12, frequency);
    }
}

void silenceBuzzer(gpio::Pwm *pwmController) { pwmController->write(0, gpio::Pwm::BUZZER); }
}  // namespace aruwlib::buzzer
