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
 
#ifndef MOTOR_HOMING_COMMAND_INTERFACE_HPP_
#define MOTOR_HOMING_COMMAND_INTERFACE_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/homeable-subsystem/homeable_subsystem_interface.hpp"

namespace aruwsrc::control
{
class MotorHomingCommandInterface : public tap::control::Command
{
public:
    MotorHomingCommandInterface(
        aruwsrc::control::HomeableSubsystemInterface& subsystem)
        : subsystem(subsystem)
    {
        addSubsystemRequirement(&subsystem);
    }

protected:
    aruwsrc::control::HomeableSubsystemInterface& subsystem;
};
}

#endif
