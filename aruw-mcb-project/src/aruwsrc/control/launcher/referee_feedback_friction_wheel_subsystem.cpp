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

#include "referee_feedback_friction_wheel_subsystem.hpp"

namespace aruwsrc::control::launcher
{
RefereeFeedbackFrictionWheelSubsystem::RefereeFeedbackFrictionWheelSubsystem(
    aruwsrc::Drivers *drivers,
    tap::motor::MotorId leftMotorId,
    tap::motor::MotorId rightMotorId,
    tap::can::CanBus canBus,
    tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismID,
    const float defaultFiringSpeed)
    : FrictionWheelSubsystem(drivers, leftMotorId, rightMotorId, canBus),
      firingSystemMechanismID(firingSystemMechanismID),
      defaultFiringSpeed(defaultFiringSpeed)
{
}

void RefereeFeedbackFrictionWheelSubsystem::refresh()
{
    FrictionWheelSubsystem::refresh();
    updatePredictedLaunchSpeed();
}

void RefereeFeedbackFrictionWheelSubsystem::updatePredictedLaunchSpeed()
{
    const float desiredLaunchSpeed = getDesiredLaunchSpeed();

    // reset averaging if desired launch speed has changed...if we change desired launch speed
    // from 15 to 30, we should predict the launch speed to be around 30, not 15.
    if (!tap::algorithms::compareFloatClose(lastDesiredLaunchSpeed, desiredLaunchSpeed, 1E-5))
    {
        lastDesiredLaunchSpeed = desiredLaunchSpeed;
        pastProjectileVelocitySpeedSummed = 0;
        ballSpeedAveragingTracker.clear();
    }

    if (drivers->refSerial.getRefSerialReceivingData())
    {
        const auto &turretData = drivers->refSerial.getRobotData().turret;

        // compute average bullet speed if new firing data received from correct mech ID
        if (prevLaunchingDataReceiveTimestamp != turretData.lastReceivedLaunchingInfoTimestamp &&
            turretData.launchMechanismID == firingSystemMechanismID)
        {
            // remove element to make room for new element
            if (ballSpeedAveragingTracker.isFull())
            {
                pastProjectileVelocitySpeedSummed -= ballSpeedAveragingTracker.getFront();
                ballSpeedAveragingTracker.removeFront();
            }

            // insert new element
            pastProjectileVelocitySpeedSummed += turretData.bulletSpeed;
            ballSpeedAveragingTracker.append(turretData.bulletSpeed);

            prevLaunchingDataReceiveTimestamp = turretData.lastReceivedLaunchingInfoTimestamp;
        }
    }
    else
    {
        pastProjectileVelocitySpeedSummed = 0;
        ballSpeedAveragingTracker.clear();
    }
}
}  // namespace aruwsrc::control::launcher
