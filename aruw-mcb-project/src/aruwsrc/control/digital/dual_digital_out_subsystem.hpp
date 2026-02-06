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
#ifndef DUAL_DIGITAL_OUT_SUBSYSTEM_HPP_
#define DUAL_DIGITAL_OUT_SUBSYSTEM_HPP_

#include "tap/communication/gpio/digital.hpp"
#include "tap/control/subsystem.hpp"

#include "digital_out_subsystem.hpp"

namespace aruwsrc::control::digital
{
class DualDigitalOutSubsystem : public DigitalOutSubsystem
{
public:
    // initial state is based on pinOne's offState
    DualDigitalOutSubsystem(
        tap::Drivers* drivers,
        tap::gpio::Digital& digital,
        const tap::gpio::Digital::OutputPin pinOne,
        const bool offStateOne,
        const tap::gpio::Digital::OutputPin pinTwo,
        const bool offStateTwo)
        : DigitalOutSubsystem(drivers, digital, pinOne, offStateOne),
          digital(digital),
          pinOne(pinOne),
          offStateOne(offStateOne),
          pinTwo(pinTwo),
          offStateTwo(offStateTwo)
    {
    }

    inline void refresh() override
    {
        digital.set(pinOne, getState() ^ offStateOne);
        digital.set(pinTwo, getState() ^ offStateTwo);
    }

    inline void refreshSafeDisconnect() override
    {
        digital.set(pinOne, offStateOne);
        digital.set(pinTwo, offStateTwo);
    }

    const char* getName() const override { return "Dual Digital Out Subsystem"; }

private:
    tap::gpio::Digital& digital;
    const tap::gpio::Digital::OutputPin pinOne;
    const bool offStateOne;
    const tap::gpio::Digital::OutputPin pinTwo;
    const bool offStateTwo;
};  // class DualDigitalOutSubsystem

}  // namespace aruwsrc::control::digital
#endif  // DUAL_DIGITAL_OUT_SUBSYSTEM_HPP_
