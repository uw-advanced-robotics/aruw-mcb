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

#include "turret_setpoint_command.hpp"

#include "aruwsrc/drivers.hpp"

using namespace tap::control::turret;

namespace aruwsrc::control::turret
{
TurretSetpointCommand::TurretSetpointCommand(
    aruwsrc::Drivers *drivers,
    TurretSubsystemInterface *turret,
    const float yawInputScalar,
    const float pitchInputScalar)
    : drivers(drivers),
      turret(turret),
      yawInputScalar(yawInputScalar),
      pitchInputScalar(pitchInputScalar)
{
    addSubsystemRequirement(turret);
}

void TurretSetpointCommand::execute()
{
    turret->setPitchSetpoint(
        turret->getPitchSetpoint() +
        pitchInputScalar * drivers->controlOperatorInterface.getTurretPitchInput());

    turret->setYawSetpoint(
        turret->getYawSetpoint() +
        yawInputScalar * drivers->controlOperatorInterface.getTurretYawInput());
}

}  // namespace aruwsrc::control::turret
