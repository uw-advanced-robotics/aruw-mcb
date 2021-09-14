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

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/turret/turret_subsystem.hpp"

#include "chassis_rel_drive.hpp"
#include "chassis_subsystem.hpp"

using namespace tap::algorithms;
using tap::Drivers;

namespace aruwsrc
{
namespace chassis
{
ChassisAutorotateCommand::ChassisAutorotateCommand(
    tap::Drivers* drivers,
    ChassisSubsystem* chassis,
    const tap::control::turret::TurretSubsystemInterface* turret)
    : drivers(drivers),
      chassis(chassis),
      turret(turret)
{
    addSubsystemRequirement(chassis);
}

void ChassisAutorotateCommand::initialize() {}

void ChassisAutorotateCommand::execute()
{
    // calculate pid for chassis rotation
    // returns a chassis rotation speed
    if (turret->isOnline())
    {
        float angleFromCenter = turret->getYawAngleFromCenter();
        float chassisRotationDesiredWheelspeed =
            chassis->chassisSpeedRotationPID(angleFromCenter, CHASSIS_AUTOROTATE_PID_KP);

        // what we will multiply x and y speed by to take into account rotation
        float rTranslationalGain =
            chassis->calculateRotationTranslationalGain(chassisRotationDesiredWheelspeed);

        float chassisXDesiredWheelspeed = limitVal(
                                              drivers->controlOperatorInterface.getChassisXInput(),
                                              -rTranslationalGain,
                                              rTranslationalGain) *
                                          ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;

        float chassisYDesiredWheelspeed = limitVal(
                                              drivers->controlOperatorInterface.getChassisYInput(),
                                              -rTranslationalGain,
                                              rTranslationalGain) *
                                          ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;
        // Rotate X and Y depending on turret angle
        rotateVector(
            &chassisXDesiredWheelspeed,
            &chassisYDesiredWheelspeed,
            -modm::toRadian(angleFromCenter));

        chassis->setDesiredOutput(
            chassisXDesiredWheelspeed,
            chassisYDesiredWheelspeed,
            chassisRotationDesiredWheelspeed);
    }
    else
    {
        ChassisRelDrive::onExecute(drivers, chassis);
    }
}

void ChassisAutorotateCommand::end(bool) { chassis->setDesiredOutput(0.0f, 0.0f, 0.0f); }

bool ChassisAutorotateCommand::isFinished() const { return false; }

}  // namespace chassis

}  // namespace aruwsrc
