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

#include "rfid_swipe_command.hpp"

#include <aruwlib/control/subsystem.hpp>

#include "rfid_swipe_subsystem.hpp"

namespace aruwsrc
{
namespace engineer
{
RfidSwipeCommand::RfidSwipeCommand(RfidSwipeSubsystem* subsystem)
    : Command(),
      grabber(subsystem)
{
    addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(grabber));
}

void RfidSwipeCommand::initialize() { grabber->setExtended(true); }

void RfidSwipeCommand::execute() {}

void RfidSwipeCommand::end(bool) { grabber->setExtended(false); }

bool RfidSwipeCommand::isFinished() const { return false; }
}  // namespace engineer

}  // namespace aruwsrc
