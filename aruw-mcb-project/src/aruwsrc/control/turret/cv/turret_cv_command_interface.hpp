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

#ifndef TURRET_CV_COMMAND_INTERFACE_HPP_
#define TURRET_CV_COMMAND_INTERFACE_HPP_

#include "tap/control/command.hpp"

// @todo: make this not a subclass of command. See
// https://aruw.slack.com/archives/C84GTRPSB/p1718012211193499
namespace aruwsrc::control::turret::cv
{
class TurretCVCommandInterface : public tap::control::Command
{
public:
    virtual bool getTurretID() const = 0;
    virtual bool isAimingWithinLaunchingTolerance() const = 0;
};
}  // namespace aruwsrc::control::turret::cv

#endif  // TURRET_CV_COMMAND_INTERFACE_HPP_
