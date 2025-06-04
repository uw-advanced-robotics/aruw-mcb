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
#ifndef SUCTION_SUBSYSTEM_HPP_
#define SUCTION_SUBSYSTEM_HPP_

#include "tap/communication/gpio/digital.hpp"
#include "tap/control/subsystem.hpp"

namespace aruwsrc::engineer
{
class SuctionSubsystem : public tap::control::Subsystem
{
public:
    SuctionSubsystem(
        tap::Drivers* drivers,
        tap::gpio::Digital& digital,
        const tap::gpio::Digital::OutputPin suckPin,
        const tap::gpio::Digital::OutputPin releasePin,
        const bool suckOffState = false,
        const bool releaseOffState = false)
        : Subsystem(drivers),
          digital(digital),
          suckPin(suckPin),
          releasePin(releasePin),
          suckOffState(suckOffState),
          releaseOffState(releaseOffState),
          suckState(suckOffState),
          releaseState(releaseOffState)
    {
    }

    inline void initialize() override {}

    inline void refresh() override
    {
        digital.set(suckPin, suckState ^ suckOffState);
        digital.set(releasePin, releaseState ^ releaseOffState);
    }

    inline void refreshSafeDisconnect() override
    {
        digital.set(suckPin, suckOffState);
        digital.set(releasePin, releaseOffState);
        set(false, false);
    }

    inline void set(bool suck, bool release)
    {
        suckState = suck;
        releaseState = release;
    }

    const char* getName() const override { return "Suction Subsystem"; }

private:
    tap::gpio::Digital& digital;
    const tap::gpio::Digital::OutputPin suckPin, releasePin;
    const bool suckOffState, releaseOffState;
    bool suckState, releaseState;
};  // class SuctionSubsystem

}  // namespace aruwsrc::engineer
#endif  // SUCTION_SUBSYSTEM_HPP_
