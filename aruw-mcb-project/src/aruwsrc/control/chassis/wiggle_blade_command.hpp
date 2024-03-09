/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef WIGGLE_BLADE_COMMAND_HPP_
#define WIGGLE_BLADE_COMMAND_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/control/comprised_command.hpp"

#include "beyblade_command.hpp"
#include "wiggle_drive_command.hpp"

namespace aruwsrc::chassis
{

class WiggleBladeCommand : public tap::control::ComprisedCommand
{
public:
    WiggleBladeCommand(
        tap::Drivers* drivers,
        HolonomicChassisSubsystem* chassis,
        const aruwsrc::control::turret::TurretMotor* yawMotor,
        aruwsrc::control::ControlOperatorInterface& operatorInterface);

    bool isReady() override;

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "Wiggle Blade command"; }

protected:
    BeybladeCommand beybladeCommand;
    WiggleDriveCommand wiggleDriveCommand;

    tap::arch::MilliTimeout beybladeTimer, wiggleTimer;

    int MIN_WIGGLE_TIME = 3000;
    int MAX_WIGGLE_TIME = 5000;

    int MIN_BEYBLADE_TIME = 1500;
    int MAX_BEYBLADE_TIME = 3000;
};

}  // namespace aruwsrc::chassis

#endif  // WIGGLE_BLADE_COMMAND_HPP_
