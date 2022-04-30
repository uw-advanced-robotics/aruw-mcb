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

#include "chassis_rel_drive.hpp"

#include "aruwsrc/drivers.hpp"

#include "chassis_subsystem.hpp"

using namespace tap::algorithms;

namespace aruwsrc::chassis
{
void ChassisRelDrive::computeDesiredUserTranslation(
    aruwsrc::Drivers *drivers,
    ChassisSubsystem *chassis,
    float chassisRotation,
    float *chassisXDesiredWheelspeed,
    float *chassisYDesiredWheelspeed)
{
    if (drivers == nullptr || chassis == nullptr || chassisXDesiredWheelspeed == nullptr ||
        chassisYDesiredWheelspeed == nullptr)
    {
        return;
    }

    const float maxWheelSpeed = ChassisSubsystem::getMaxWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

    // what we will multiply x and y speed by to take into account rotation
    float rotationLimitedMaxTranslationalSpeed =
        chassis->calculateRotationTranslationalGain(chassisRotation) * maxWheelSpeed;

    *chassisXDesiredWheelspeed = limitVal(
        drivers->controlOperatorInterface.getChassisXInput(),
        -rotationLimitedMaxTranslationalSpeed,
        rotationLimitedMaxTranslationalSpeed);

    *chassisYDesiredWheelspeed = limitVal(
        drivers->controlOperatorInterface.getChassisYInput(),
        -rotationLimitedMaxTranslationalSpeed,
        rotationLimitedMaxTranslationalSpeed);
}

void ChassisRelDrive::onExecute(aruwsrc::Drivers *drivers, ChassisSubsystem *chassis)
{
    float chassisRotationDesiredWheelspeed = drivers->controlOperatorInterface.getChassisRInput();

    float chassisXDesiredWheelspeed = 0.0f;
    float chassisYDesiredWheelspeed = 0.0f;

    computeDesiredUserTranslation(
        drivers,
        chassis,
        chassisRotationDesiredWheelspeed,
        &chassisXDesiredWheelspeed,
        &chassisYDesiredWheelspeed);

    chassis->setDesiredOutput(
        chassisXDesiredWheelspeed,
        chassisYDesiredWheelspeed,
        chassisRotationDesiredWheelspeed);
}
}  // namespace aruwsrc::chassis
