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

#ifndef RFID_SWIPE_COMMAND_HPP_
#define RFID_SWIPE_COMMAND_HPP_

#include <aruwlib/control/command.hpp>

namespace aruwsrc
{
namespace engineer
{
class RfidSwipeSubsystem;

class RfidSwipeCommand : public aruwlib::control::Command
{
public:
    explicit RfidSwipeCommand(RfidSwipeSubsystem* subsystem);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "rfid swipe command"; }

private:
    RfidSwipeSubsystem* grabber;
};  // class SqueezeGrabberCommand

}  // namespace engineer

}  // namespace aruwsrc
#endif  // RFID_SWIPE_COMMAND_HPP_
