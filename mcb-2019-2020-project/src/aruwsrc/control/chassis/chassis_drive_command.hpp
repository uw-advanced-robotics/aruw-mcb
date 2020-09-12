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

#ifndef __CHASSIS_DRIVE_COMMAND_HPP__
#define __CHASSIS_DRIVE_COMMAND_HPP__

#include <aruwlib/Drivers.hpp>
#include <aruwlib/control/command.hpp>

#include "chassis_subsystem.hpp"

namespace aruwsrc
{
namespace chassis
{
class ChassisDriveCommand : public aruwlib::control::Command
{
public:
    ChassisDriveCommand(aruwlib::Drivers* drivers, ChassisSubsystem* chassis)
        : drivers(drivers),
          chassis(chassis)
    {
        addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(chassis));
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "chassis drive command"; }

private:
    aruwlib::Drivers* drivers;
    ChassisSubsystem* chassis;
};

}  // namespace chassis

}  // namespace aruwsrc

#endif
