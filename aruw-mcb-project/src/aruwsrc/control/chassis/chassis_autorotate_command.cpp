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

#include "chassis_autorotate_command.hpp"

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/communication/remote.hpp"
#include "aruwlib/drivers.hpp"

#include "aruwsrc/control/turret/turret_subsystem.hpp"

#include "chassis_subsystem.hpp"

using aruwlib::Drivers;

namespace aruwsrc
{
namespace chassis
{
ChassisAutorotateCommand::ChassisAutorotateCommand(
    aruwlib::Drivers* drivers,
    ChassisSubsystem* chassis,
    const aruwlib::control::turret::TurretSubsystemInterface* turret,
    float chassisAutorotatePidKp)
    : drivers(drivers),
      chassis(chassis),
      turret(turret),
      CHASSIS_AUTOROTATE_PID_KP(chassisAutorotatePidKp)
{
    addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(chassis));
}

void ChassisAutorotateCommand::initialize() {}

void ChassisAutorotateCommand::execute()
{
    const float maxWheelSpeed = chassis->getMaxWheelSpeedSingleMotor();

    // calculate pid for chassis rotation
    // returns a chassis rotation speed
    float chassisRotationDesiredWheelspeed;
    if (turret->isOnline())
    {
        chassisRotationDesiredWheelspeed = chassis->chassisSpeedRotationPID(
            turret->getYawAngleFromCenter(),
            CHASSIS_AUTOROTATE_PID_KP);
    }
    else
    {
        chassisRotationDesiredWheelspeed =
            drivers->controlOperatorInterface.getChassisRInput() * maxWheelSpeed;
    }

    // what we will multiply x and y speed by to take into account rotation
    float rTranslationalGain =
        chassis->calculateRotationTranslationalGain(chassisRotationDesiredWheelspeed);

    float chassisXDesiredWheelspeed = aruwlib::algorithms::limitVal<float>(
                                          drivers->controlOperatorInterface.getChassisXInput(),
                                          -rTranslationalGain,
                                          rTranslationalGain) *
                                      maxWheelSpeed;

    float chassisYDesiredWheelspeed = aruwlib::algorithms::limitVal<float>(
                                          drivers->controlOperatorInterface.getChassisYInput(),
                                          -rTranslationalGain,
                                          rTranslationalGain) *
                                      maxWheelSpeed;

    // Rotate X and Y depending on turret angle
    aruwlib::algorithms::rotateVector(
        &chassisXDesiredWheelspeed,
        &chassisYDesiredWheelspeed,
        -aruwlib::algorithms::degreesToRadians(turret->getYawAngleFromCenter()));

    chassis->setDesiredOutput(
        chassisXDesiredWheelspeed,
        chassisYDesiredWheelspeed,
        chassisRotationDesiredWheelspeed);
}

void ChassisAutorotateCommand::end(bool) { chassis->setDesiredOutput(0.0f, 0.0f, 0.0f); }

bool ChassisAutorotateCommand::isFinished() const { return false; }

}  // namespace chassis

}  // namespace aruwsrc
