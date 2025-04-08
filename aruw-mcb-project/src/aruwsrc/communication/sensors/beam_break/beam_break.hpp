/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BEAM_BREAK_HPP_
#define BEAM_BREAK_HPP_

#include "tap/communication/gpio/analog.hpp"
#include "tap/communication/gpio/digital.hpp"
#include "tap/communication/sensors/limit_switch/limit_switch_interface.hpp"

namespace aruwsrc::communication::sensors::beam_break
{
/**
 * Implements an analog input beam break sensor. The sensor is considered to be triggered when the
 * analog input value is above the threshold. This can be inverted to trigger when the value is
 * below the threshold.
 */
class AnalogBeamBreak : public tap::communication::sensors::limit_switch::LimitSwitchInterface
{
private:
    const tap::gpio::Analog* analog;
    const tap::gpio::Analog::Pin pin;
    const uint16_t threshold;
    bool inverted;

public:
    AnalogBeamBreak(
        tap::gpio::Analog* analog,
        tap::gpio::Analog::Pin pin,
        uint16_t threshold,
        bool inverted = false)
        : analog(analog),
          pin(pin),
          threshold(threshold),
          inverted(inverted)
    {
    }

    bool getLimitSwitchDepressed() const override
    {
        if (inverted)
        {
            return analog->read(pin) <= threshold;
        }
        else
        {
            return analog->read(pin) >= threshold;
        }
    }
};

/**
 * Implements a digital input beam break sensor. The sensor is considered to be triggered when the
 * digital input value is high. This can be inverted to trigger when the value is low.
 */
class DigitalBeamBreak : public tap::communication::sensors::limit_switch::LimitSwitchInterface
{
private:
    const tap::gpio::Digital* digital;
    const tap::gpio::Digital::InputPin pin;
    bool inverted;

public:
    DigitalBeamBreak(
        tap::gpio::Digital* digital,
        tap::gpio::Digital::InputPin pin,
        bool inverted = false)
        : digital(digital),
          pin(pin),
          inverted(inverted)
    {
    }

    bool getLimitSwitchDepressed() const override
    {
        if (inverted)
        {
            return !digital->read(pin);
        }
        else
        {
            return digital->read(pin);
        }
    }
};

}  // namespace aruwsrc::communication::sensors::beam_break

#endif  // BEAM_BREAK_HPP_
