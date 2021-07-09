/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef RFID_SWIPE_SUBSYSTEM_
#define RFID_SWIPE_SUBSYSTEM_

#include "aruwlib/communication/gpio/digital.hpp"
#include "aruwlib/control/subsystem.hpp"

namespace aruwsrc
{
namespace engineer
{
/**
 * This is a subsystem code for Engineer RFID reader subsystem.
 * The RFID reader will be actuated by a single solenoid that
 * controls one pneumatic piston.
 */
class RfidSwipeSubsystem : public aruwlib::control::Subsystem
{
public:
    RfidSwipeSubsystem(aruwlib::Drivers *drivers, aruwlib::gpio::Digital::OutputPin pin)
        : aruwlib::control::Subsystem(drivers),
          pin(pin),
          isRfidExtended(false)
    {
    }

    void setExtended(bool isRfidExtended);

    bool isExtended() const;

private:
    aruwlib::gpio::Digital::OutputPin pin;

    bool isRfidExtended;
};  // RfidSwipeSubsystem

}  // namespace engineer

}  // namespace aruwsrc

#endif  // RFID_SWIPE_SUBSYSTEM_