/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef ATTACH_CONTROLLER_COMMAND_HPP_
#define ATTACH_CONTROLLER_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "../balstd_chassis_subsystem.hpp"

#include "chassis_controller_interface.hpp"

namespace aruwsrc::control::balstd
{

class AttachControllerCommand : public tap::control::Command
{
public:
    AttachControllerCommand(
        BalstdChassisSubsystem& chassis,
        BalstdChassisControllerInterface* controller)
        : chassis(chassis),
          controller(controller)
    {
        this->addSubsystemRequirement(&chassis);
    }

    void initialize() override { chassis.attachController(controller); }

    void execute() override {}

    void end(bool interrupted) override {}

    bool isFinished() const override { return true; }

    const char* getName() const override { return "Attach Controller Command"; }

private:
    BalstdChassisSubsystem& chassis;
    BalstdChassisControllerInterface* controller;
};  // class AttachControllerCommand

}  // namespace aruwsrc::control::balstd
#endif  // ATTACH_CONTROLLER_COMMAND_HPP_
