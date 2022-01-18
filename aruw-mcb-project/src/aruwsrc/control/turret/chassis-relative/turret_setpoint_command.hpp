/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TURRET_SETPOINT_COMMAND_HPP_
#define TURRET_SETPOINT_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/control/turret_subsystem_interface.hpp"

#include "aruwsrc/drivers.hpp"

namespace aruwsrc::control::turret
{
/**
 * A command that changes the setpoint of an associated turret. Interfaces with a generic turret.
 * Sets the turret's pitch and yaw setpoints based on control operator interface input.
 */
class TurretSetpointCommand : public tap::control::Command
{
public:
    TurretSetpointCommand(
        aruwsrc::Drivers *drivers,
        tap::control::turret::TurretSubsystemInterface *turret,
        const float yawInputScalar,
        const float pitchInputScalar);

    bool isReady() override { return turret->isOnline(); }

    void initialize() override {}

    bool isFinished() const override { return false; }

    void execute() override;

    void end(bool) override {}

    const char *getName() const override { return "turret setpoint control"; }

private:
    aruwsrc::Drivers *drivers;

    tap::control::turret::TurretSubsystemInterface *turret;

    const float yawInputScalar;
    const float pitchInputScalar;
};  // class TurretSetpointCommand

}  // namespace aruwsrc::control::turret

#endif  // TURRET_SETPOINT_COMMAND_HPP_
