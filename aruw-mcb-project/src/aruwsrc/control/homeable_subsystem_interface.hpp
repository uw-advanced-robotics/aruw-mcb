/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef HOMEABLE_SUBSYSTEM_INTERFACE_HPP_
#define HOMEABLE_SUBSYSTEM_INTERFACE_HPP_

#include "tap/control/subsystem.hpp"

namespace aruwsrc::control
{
class HomeableSubsystemInterface : public tap::control::Subsystem
{
public:
    virtual void setMotorOutput(int32_t desiredOutput) = 0;
    virtual bool isStalled() const = 0;
    virtual void setLowerBound() = 0;
    virtual void setUpperBound() = 0;
};
}  // namespace aruwsrc::control

#endif