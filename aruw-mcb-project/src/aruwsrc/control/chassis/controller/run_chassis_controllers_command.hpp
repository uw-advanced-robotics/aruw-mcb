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
#ifndef RUN_CHASSIS_CONTROLLERS_HPP_
#define RUN_CHASSIS_CONTROLLERS_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/control/chassis/controller/chassis_yaw_controller_interface.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"

namespace aruwsrc::control::chassis::controller
{

class RunChassisControllersCommand : public tap::control::Command
{
public:
    RunChassisControllersCommand(
        aruwsrc::control::chassis::HolonomicChassisSubsystem& chassis,
        ChassisTranslationControllerInterface* translationController,
        ChassisYawControllerInterface* yawController)
        : chassis(chassis),
          translationController(translationController),
          yawController(yawController)
    {
        this->addSubsystemRequirement(&chassis);
    }

    void initialize() override
    {
        chassis.attachTranslationController(translationController);
        chassis.attachYawController(yawController);
    };

    void execute() override {};

    void end(bool interrupted) override
    {
        chassis.attachTranslationController(nullptr);
        chassis.attachYawController(nullptr);
    };

    bool isFinished() const override
    {
        // TODO: this is wrong if one never finishes and only the other has a duration
        bool translationFinished =
            translationController ? translationController->isFinished() : true;
        bool yawFinished = yawController ? yawController->isFinished() : true;
        return translationFinished && translationController;
    };

    const char* getName() const override { return "Attach Chassis Yaw Controller Command"; }

private:
    aruwsrc::control::chassis::HolonomicChassisSubsystem& chassis;
    ChassisTranslationControllerInterface* translationController;
    ChassisYawControllerInterface* yawController;
};  // class RunChassisControllersCommand

}  // namespace aruwsrc::control::chassis::controller
#endif  // RUN_CHASSIS_CONTROLLERS_HPP_
