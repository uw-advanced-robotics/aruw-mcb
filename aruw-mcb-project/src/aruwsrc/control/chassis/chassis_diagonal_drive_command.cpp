/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "chassis_diagonal_drive_command.hpp"

#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/chassis_rel_drive.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "aruwsrc/control/chassis/new-chassis/chassis_subsystem.hpp"

namespace aruwsrc::chassis
{
ChassisDiagonalDriveCommand::ChassisDiagonalDriveCommand(
    tap::Drivers* drivers,
    aruwsrc::control::ControlOperatorInterface* operatorInterface,
    ChassisSubsystem* chassis,
    const aruwsrc::control::turret::TurretMotor* yawMotor,
    ChassisSymmetry chassisSymmetry)
    : ChassisAutorotateCommand(drivers, operatorInterface, chassis, yawMotor, chassisSymmetry)
{
    // subsystem requirement added by base class
    // ensures that we are 90 degree symmetrical. This only works for holonomic chasses.
    assert(chassisSymmetry == ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_90);
}

float ChassisDiagonalDriveCommand::computeAngleFromCenterForAutorotation(
    float turretAngleFromCenter,
    float maxAngleFromCenter)
{
    float angleFromCenterForChassisAutorotate = 0.0f;

    if (const auto chassisVelocity = chassis->getActualVelocityChassisRelative();
        hypot(chassisVelocity[0][0], chassisVelocity[1][0]) > AUTOROTATION_DIAGONAL_SPEED &&
        !operatorInterface->isSlowMode())
    {
        angleFromCenterForChassisAutorotate =
            ContiguousFloat(turretAngleFromCenter, -M_PI_2, M_PI_2).getValue() + M_PI_4;
    }
    else
    {
        angleFromCenterForChassisAutorotate =
            ContiguousFloat(turretAngleFromCenter, -maxAngleFromCenter, maxAngleFromCenter)
                .getValue();
    }
    return angleFromCenterForChassisAutorotate;
}

}  // namespace aruwsrc::chassis
