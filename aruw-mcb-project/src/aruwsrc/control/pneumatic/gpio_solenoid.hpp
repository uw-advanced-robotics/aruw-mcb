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

#ifndef GPIO_SOLENOID_HPP_
#define GPIO_SOLENOID_HPP_

#include "tap/drivers.hpp"

namespace aruwsrc::control::pneumatic
{
enum class SolenoidState
{
    OFF,
    EXTENDED
};
/**
 * A class for controlling a solenoid using GPIO pins.
 */
class GpioSolenoid
{
public:
    GpioSolenoid(tap::Drivers* drivers, tap::gpio::Digital::OutputPin extendPin);

    void extend();

    void off();

    SolenoidState getState() { return state; }

private:
    tap::Drivers* drivers;
    tap::gpio::Digital::OutputPin extendPin;
    SolenoidState state;
};
}  // namespace aruwsrc::control::pneumatic

#endif
