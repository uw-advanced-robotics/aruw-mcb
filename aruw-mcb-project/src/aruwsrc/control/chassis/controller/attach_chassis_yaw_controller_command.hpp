/*
 * Copyright (c) 2026-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef ATTACH_CHASSIS_YAW_CONTROLLER_COMMAND_HPP_
#define ATTACH_CHASSIS_YAW_CONTROLLER_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/control/chassis/controller/chassis_yaw_controller_interface.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"

namespace aruwsrc::control::chassis::controller
{

class AttachChassisYawControllerCommand : public tap::control::Command
{
public:
    AttachChassisYawControllerCommand(
        aruwsrc::control::chassis::HolonomicChassisSubsystem& chassis,
        ChassisYawControllerInterface* controller)
        : chassis(chassis),
          controller(controller)
    {
        this->addSubsystemRequirement(&chassis);
    }

    void initialize() override { chassis.attachYawController(controller); };

    void execute() override {};

    void end(bool interrupted) override {};

    bool isFinished() const override { return true; };

    const char* getName() const override { return "Attach Chassis Yaw Controller Command"; }

private:
    aruwsrc::control::chassis::HolonomicChassisSubsystem& chassis;
    ChassisYawControllerInterface* controller;
};  // class AttachChassisYawControlerCommand

}  // namespace aruwsrc::control::chassis::controller
#endif  // ATTACH_CHASSIS_YAW_CONTROLLER_COMMAND_HPP_
