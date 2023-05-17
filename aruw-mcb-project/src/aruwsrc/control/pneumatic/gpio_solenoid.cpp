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

#include "gpio_double_solenoid.hpp"

namespace aruwsrc::control::pneumatic
{
GpioSolenoid::GpioSolenoid(tap::Drivers* drivers, tap::gpio::Digital::OutputPin extendPin)
    : drivers(drivers),
      extendPin(extendPin)
{
    state = SolenoidState::OFF;
}

void GpioSolenoid::extend()
{
    drivers->digital.set(extendPin, true);
    state = SolenoidState::EXTENDED;
}

void GpioSolenoid::off()
{
    drivers->digital.set(extendPin, false);
    state = SolenoidState::OFF;
}

}  // namespace aruwsrc::control::pneumatic
