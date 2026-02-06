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
#ifndef DIGITAL_OUT_SUBSYSTEM_HPP_
#define DIGITAL_OUT_SUBSYSTEM_HPP_

#include "tap/communication/gpio/digital.hpp"
#include "tap/control/subsystem.hpp"

namespace aruwsrc::engineer
{
class DigitalOutSubsystem : public tap::control::Subsystem
{
public:
    // initial state is based on offStateOne
    DigitalOutSubsystem(
        tap::Drivers* drivers,
        tap::gpio::Digital& digital,
        const tap::gpio::Digital::OutputPin pinOne,
        const bool offStateOne = true,
        const tap::gpio::Digital::OutputPin* pinTwo = nullptr,
        const bool offStateTwo = false, 
        const bool initialState = false)
        : Subsystem(drivers),
          digital(digital),
          pinOne(pinOne),
          pinTwo(pinTwo),
          offStateOne(offStateOne),
          offStateTwo(offStateTwo),
          state(initialState), 
    {
    }

    inline void initialize() override {}

    inline void refresh() override
    {
        digital.set(pinOne, state ^ offStateOne);
        digital.set(pinTwo, state ^ offStateTwo);
    }

    inline bool getState() { return state; }

    inline void refreshSafeDisconnect() override
    {
        digital.set(pinOne, offStateOne);
        digital.set(pinTwo, offStateTwo);
    }

    inline void set(bool s) { state = s; }

    const char* getName() const override { return "Digital Out Subsystem"; }

private:
    tap::gpio::Digital& digital;
    const tap::gpio::Digital::OutputPin pinOne;
    const tap::gpio::Digital::OutputPin* pinTwo;
    const bool offStateOne;
    const bool offStateTwo;
    bool state;
};  // class DigitalOutSubsystem

}  // namespace aruwsrc::engineer
#endif  // DIGITAL_OUT_SUBSYSTEM_HPP_
