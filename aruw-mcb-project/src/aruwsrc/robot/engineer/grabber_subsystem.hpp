/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/communication/gpio/digital.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/util_macros.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc
{
namespace engineer
{
/**
 * This is a subsystem code for Engineer grabber mechanism.
 * The grabber will be actuated by a single solenoid that
 * controls two pneumatic pistons.
 */
class GrabberSubsystem : public tap::control::Subsystem
{
public:
    GrabberSubsystem(tap::Drivers *drivers, tap::gpio::Digital::OutputPin pin);

    mockable void setSqueezed(bool isGrabberSqueezed);

    mockable bool isSqueezed() const;

    const char *getName() const override { return "Grabber"; }

private:
    tap::gpio::Digital::OutputPin pin;

    bool isGrabberSqueezed;

    uint64_t testTime;

};  // GrabberSubsystem

}  // namespace engineer

}  // namespace aruwsrc

#endif  // GRABBER_SUBSYSTEM_HPP_
