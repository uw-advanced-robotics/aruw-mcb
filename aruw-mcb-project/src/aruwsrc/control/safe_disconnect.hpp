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

#ifndef SAFE_DISCONNECT_HPP_
#define SAFE_DISCONNECT_HPP_

#include "tap/control/command_scheduler.hpp"
#include "tap/drivers.hpp"

/**
 * Defines the condition for a robot to be "safely disconnected" to be
 * when the remote is disconnected. Ends running of all current Commands and
 * disallows new Commands from being added.
 */
namespace aruwsrc::control
{
class RemoteSafeDisconnectFunction : public tap::control::SafeDisconnectFunction
{
public:
    RemoteSafeDisconnectFunction(tap::Drivers *drivers);
    virtual bool operator()();

private:
    tap::Drivers *drivers;
};
}  // namespace aruwsrc::control

#endif  // SAFE_DISCONNECT_HPP_
