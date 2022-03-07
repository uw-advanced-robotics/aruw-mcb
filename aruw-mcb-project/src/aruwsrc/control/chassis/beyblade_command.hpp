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

#ifndef BEYBLADE_COMMAND_HPP_
#define BEYBLADE_COMMAND_HPP_

#include "tap/algorithms/ramp.hpp"
#include "tap/control/command.hpp"
#include "tap/control/turret_subsystem_interface.hpp"

#include "modm/math/interpolation/linear.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::chassis
{
class ChassisSubsystem;

/**
 * A command that automatically rotates the chassis while maintaining turret angle
 */
class BeybladeCommand : public tap::control::Command
{
public:
    BeybladeCommand(
        aruwsrc::Drivers* drivers,
        ChassisSubsystem* chassis,
        const tap::control::turret::TurretSubsystemInterface* turret);

    /**
     * Sets rotational input target on Ramp
     */
    void initialize() override;

    /**
     * Updates rotational speed with ramp, translates and rotates x and y inputs based on turret
     * angle Sets chassis motor outputs
     */
    void execute() override;

    void end(bool) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "chassis beyblade"; }

private:
    float rampTarget;
    float rotationDirection;

    tap::algorithms::Ramp rotateSpeedRamp;

    aruwsrc::Drivers* drivers;
    ChassisSubsystem* chassis;
    const tap::control::turret::TurretSubsystemInterface* turret;
};  // class BeybladeCommand

}  // namespace aruwsrc::chassis

#endif  // BEYBLADE_COMMAND_HPP_
