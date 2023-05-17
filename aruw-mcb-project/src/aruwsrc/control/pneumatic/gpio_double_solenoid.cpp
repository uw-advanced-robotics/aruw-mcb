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
GpioDoubleSolenoid::GpioDoubleSolenoid(
    tap::Drivers* drivers,
    tap::gpio::Digital::OutputPin extendPin,
    tap::gpio::Digital::OutputPin retractPin)
    : drivers(drivers),
      extensionSolenoid(drivers, extendPin),
      retractionSolenoid(drivers, retractPin)
{
    state = DoubleSolenoidState::OFF;
}

void GpioDoubleSolenoid::extend()
{
    extensionSolenoid.extend();
    retractionSolenoid.off();
    state = DoubleSolenoidState::EXTENDED;
}

void GpioDoubleSolenoid::retract()
{
    extensionSolenoid.off();
    retractionSolenoid.extend();
    state = DoubleSolenoidState::RETRACTED;
}

void GpioDoubleSolenoid::off()
{
    extensionSolenoid.off();
    retractionSolenoid.off();
    state = DoubleSolenoidState::OFF;
}

}  // namespace aruwsrc::control::pneumatic
