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
#ifndef NARY_DIGITAL_OUT_SUBSYSTEM_HPP_
#define NARY_DIGITAL_OUT_SUBSYSTEM_HPP_

#include "tap/communication/gpio/digital.hpp"
#include "tap/control/subsystem.hpp"

#include "digital_out_subsystem.hpp"

namespace aruwsrc::control::digital
{
template <uint16_t NUM_PINS>
class NaryDigitalOutSubsystem : public DigitalOutSubsystem
{
public:
    /** Creates a new NaryDigitalOutSubsystem to handle n-number digital out pins tied to the same
     * state
     * @param[in] drivers reference to robot's drivers object
     * @param[in] pins array of digital output pins to be controlled of size NUM_PINS
     * @param[in] offStates state pins should be in when off, based on order of pins. Also of size
     * NUM_PINS offStates[0] is the initial state of the subsystem so be intentional
     */
    NaryDigitalOutSubsystem(
        tap::Drivers* drivers,
        tap::gpio::Digital& digital,
        const tap::gpio::Digital::OutputPin pins[NUM_PINS],
        const bool offStates[NUM_PINS])
        : DigitalOutSubsystem(drivers, digital, pins[0], offStates[0]),
          digital(digital),
          pins(pins),
          offStates(offStates)
    {
    }

    inline void refresh() override
    {
        for (int i = 0; i < NUM_PINS; i++)
        {
            digital.set(pins[i], getState() ^ offStates[i]);
        }
    }

    inline void refreshSafeDisconnect() override
    {
        for (int i = 0; i < NUM_PINS; i++)
        {
            digital.set(pins[i], offStates[i]);
        }
    }

    const char* getName() const override { return "Nary Digital Out Subsystem"; }

private:
    tap::gpio::Digital& digital;
    const tap::gpio::Digital::OutputPin pins[NUM_PINS];
    const bool offStates[NUM_PINS];
};  // class NaryDigitalOutSubsystem

}  // namespace aruwsrc::control::digital
#endif  // NARY_DIGITAL_OUT_SUBSYSTEM_HPP_
