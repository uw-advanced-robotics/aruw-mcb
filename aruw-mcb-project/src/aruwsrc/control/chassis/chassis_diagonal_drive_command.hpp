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

#ifndef CHASSIS_DIAGONAL_DRIVE_COMMAND_HPP_
#define CHASSIS_DIAGONAL_DRIVE_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/turret/turret_motor.hpp"
#include "aruwsrc/robot/control_operator_interface.hpp"

#include "chassis_autorotate_command.hpp"

using namespace tap::algorithms;

namespace aruwsrc::chassis
{
class ChassisSubsystem;

/**
 * A command that continuously attempts to rotate the chasis so that the turret is
 * aligned with the center of the chassis.
 * Diagonal drive rotates the chassis to align along a diagonal while ground speed above
 * AUTOROTATION_DIAGONAL_SPEED. This improves power efficiency and stability.
 */
class ChassisDiagonalDriveCommand : public ChassisAutorotateCommand
{
public:
    ChassisDiagonalDriveCommand(
        tap::Drivers* drivers,
        aruwsrc::control::ControlOperatorInterface* operatorInterface,
        ChassisSubsystem* chassis,
        const aruwsrc::control::turret::TurretMotor* yawMotor,
        ChassisSymmetry chassisSymmetry);

    const char* getName() const override { return "chassis diagonal drive"; }

private:
    float computeAngleFromCenterForAutorotation(
        float turretAngleFromCenter,
        float maxAngleFromCenter) override;

};  // class ChassisAutorotateCommand

}  // namespace aruwsrc::chassis

#endif  // CHASSIS_AUTOROTATE_COMMAND_HPP_
