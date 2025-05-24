/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ENGINEER_MANUAL_DRIVE_COMMAND_HPP_
#define ENGINEER_MANUAL_DRIVE_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/robot/engineer/engineer_control_operator_interface.hpp"

using namespace aruwsrc::control::engineer;

namespace aruwsrc::engineer::chassis
{
/**
 * A command that controls chassis-relative mecanum drive.
 */
class EngineerManualDriveCommand : public tap::control::Command
{
public:
    EngineerManualDriveCommand(
        tap::Drivers* drivers,
        EngineerControlOperatorInterface* operatorInterface,
        aruwsrc::chassis::HolonomicChassisSubsystem* chassis);

    void initialize() override;

    /**
     * Gets remote x, y, and r commands, limits them, applies a rotation ratio between [0, 1]
     * that is inversely proportional to the rotation component to the x and y components of
     * movement, and sets `setDesiredOutput` with the scaled <x, y, r> components.
     */
    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "chassis drive"; }

private:
    tap::Drivers* drivers;
    EngineerControlOperatorInterface* operatorInterface;
    aruwsrc::chassis::HolonomicChassisSubsystem* chassis;
    aruwsrc::chassis::ChassisDriveCommand chassisDriveCommand;
};  // class EngineerManualDriveCommand

}  // namespace aruwsrc::engineer::chassis

#endif  // ENGINEER_MANUAL_DRIVE_COMMAND_HPP_
