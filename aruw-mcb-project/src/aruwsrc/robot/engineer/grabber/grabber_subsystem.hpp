/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef GRABBER_SUBSYSTEM_HPP_
#define GRABBER_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

#include "aruwsrc/control/pneumatic/gpio_solenoid.hpp"

namespace aruwsrc::engineer::grabber
{

class GrabberSubsystem : public tap::control::Subsystem
{
public:
    GrabberSubsystem(
        tap::Drivers* drivers,
        aruwsrc::control::pneumatic::GpioSolenoid* grabberSolenoid);

    void initialize() override {}

    void refresh() override {}

    void refreshSafeDisconnect() override { pump->off(); }

    void toggle()
    {
        if (pump->getState() == aruwsrc::control::pneumatic::SolenoidState::OFF)
        {
            pump->extend();
        }
        else
        {
            pump->off();
        }
    }

    const char* getName() const override { return "Grabber"; }

private:
    aruwsrc::control::pneumatic::GpioSolenoid* pump;
};  // class GrabberSubsystem

}  // namespace aruwsrc::engineer::grabber
#endif  // GRABBER_SUBSYSTEM_HPP_
