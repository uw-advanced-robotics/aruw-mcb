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

#ifndef GPIO_DOUBLE_SOLENOID_HPP_
#define GPIO_DOUBLE_SOLENOID_HPP_

#include "tap/drivers.hpp"

#include "gpio_solenoid.hpp"

namespace aruwsrc::control::pneumatic
{
enum class DoubleSolenoidState
{
    OFF,
    EXTENDED,
    RETRACTED
};
/**
 * A class for controlling a double solenoid using GPIO pins.
 */
class GpioDoubleSolenoid
{
public:
    GpioDoubleSolenoid(
        tap::Drivers* drivers,
        tap::gpio::Digital::OutputPin extendPin,
        tap::gpio::Digital::OutputPin retractPin);

    void extend();
    void retract();
    void off();
    DoubleSolenoidState getState() { return state; }

private:
    tap::Drivers* drivers;
    GpioSolenoid extensionSolenoid;
    GpioSolenoid retractionSolenoid;
    DoubleSolenoidState state;
};
}  // namespace aruwsrc::control::pneumatic

#endif
