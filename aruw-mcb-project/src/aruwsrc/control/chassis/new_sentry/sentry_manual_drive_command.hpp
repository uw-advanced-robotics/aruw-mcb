/*
 * Copyright (c) 2021-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SENTRY_MANUAL_DRIVE_COMMAND_HPP_
#define SENTRY_MANUAL_DRIVE_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_control_operator_interface.hpp"

using namespace aruwsrc::control::sentry;

namespace aruwsrc
{
namespace control::sentry
{
/**
 * A command that controls chassis-relative mecanum drive.
 */
class SentryManualDriveCommand : public tap::control::Command
{
public:
    SentryManualDriveCommand(
        tap::Drivers* drivers,
        SentryControlOperatorInterface* operatorInterface,
        chassis::ChassisSubsystem* chassis);

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
    SentryControlOperatorInterface* operatorInterface;
    chassis::ChassisSubsystem* chassis;
};  // class SentryManualDriveCommand

}  // namespace control::sentry

}  // namespace aruwsrc

#endif  // SENTRY_MANUAL_DRIVE_COMMAND_HPP_
