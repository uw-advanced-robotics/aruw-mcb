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

#ifndef MATCH_RUNNING_GOVERNOR_HPP_
#define MATCH_RUNNING_GOVERNOR_HPP_

#include <cstdint>

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/control/governor/command_governor_interface.hpp"

// @todo namespace????
/**
 * A governor that when triggered pauses the ability to schedule an associated Command for a
 * specified period.
 */
class MatchRunningGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    MatchRunningGovernor(const tap::communication::serial::RefSerial& refSerial) : refSerial(refSerial)
    {
    }

    bool isReady() override { return (refSerial.getGameData().gameStage == tap::communication::serial::RefSerial::Rx::GameStage::IN_GAME); }

    bool isFinished() override { return false; }

private:
    const tap::communication::serial::RefSerial& refSerial;
};

#endif  // MATCH_RUNNING_GOVERNOR_HPP_
