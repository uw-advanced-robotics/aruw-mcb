/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef YELLOW_CARDED_GOVERNOR_HPP_
#define YELLOW_CARDED_GOVERNOR_HPP_

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/control/command_governor_interface.hpp"

namespace aruwsrc::control::governor
{
class YellowCardedGovernor : public tap::control::CommandGovernorInterface
{
public:
    YellowCardedGovernor(tap::communication::serial::RefSerial &refSerial) : refSerial(refSerial) {}

    bool isReady() final { return refSerial.operatorBlinded(); }

    bool isFinished() final { return !refSerial.operatorBlinded(); }

private:
    tap::communication::serial::RefSerial &refSerial;
};
}  // namespace aruwsrc::agitator

#endif  // YELLOW_CARDED_GOVERNOR_HPP_
