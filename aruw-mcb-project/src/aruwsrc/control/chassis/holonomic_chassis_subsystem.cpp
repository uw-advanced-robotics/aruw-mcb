/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "holonomic_chassis_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

using namespace tap::algorithms;

namespace aruwsrc
{
namespace chassis
{
modm::Pair<int, float> HolonomicChassisSubsystem::lastComputedMaxWheelSpeed =
    CHASSIS_POWER_TO_MAX_SPEED_LUT[0];

HolonomicChassisSubsystem::HolonomicChassisSubsystem(
    tap::Drivers* drivers,
    tap::communication::sensors::current::CurrentSensorInterface* currentSensor,
        can::capbank::CapacitorBank* capacitorBank)
    : tap::control::chassis::ChassisSubsystemInterface(drivers),
      currentSensor(currentSensor),
      chassisPowerLimiter(
          drivers,
          currentSensor,
          capacitorBank,
          STARTING_ENERGY_BUFFER,
          ENERGY_BUFFER_LIMIT_THRESHOLD,
          ENERGY_BUFFER_CRIT_THRESHOLD)
{
}

// HolonomicChassisSubsystem::~HolonomicChassisSubsystem() {}

float HolonomicChassisSubsystem::chassisSpeedRotationPID(float currentAngleError, float errD)
{
    // P
    float currRotationPidP = currentAngleError * AUTOROTATION_PID_KP;
    currRotationPidP = limitVal(currRotationPidP, -AUTOROTATION_PID_MAX_P, AUTOROTATION_PID_MAX_P);

    // D
    float currentRotationPidD = errD * AUTOROTATION_PID_KD;

    currentRotationPidD =
        limitVal(currentRotationPidD, -AUTOROTATION_PID_MAX_D, AUTOROTATION_PID_MAX_D);

    float wheelRotationSpeed = limitVal(
        currRotationPidP + currentRotationPidD,
        -AUTOROTATION_PID_MAX_OUTPUT,
        AUTOROTATION_PID_MAX_OUTPUT);

    return wheelRotationSpeed;
}

float HolonomicChassisSubsystem::calculateRotationTranslationalGain(
    float chassisRotationDesiredWheelspeed)
{
    // what we will multiply x and y speed by to take into account rotation
    float rTranslationalGain = 1.0f;

    // the x and y movement will be slowed by a fraction of auto rotation amount for maximizing
    // power consumption when the wheel rotation speed for chassis rotation is greater than the
    // MIN_ROTATION_THRESHOLD
    if (fabsf(chassisRotationDesiredWheelspeed) > MIN_ROTATION_THRESHOLD)
    {
        const float maxWheelSpeed = HolonomicChassisSubsystem::getMaxWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

        // power(max revolve speed + min rotation threshold - specified revolve speed, 2) /
        // power(max revolve speed, 2)
        rTranslationalGain = powf(
            (maxWheelSpeed + MIN_ROTATION_THRESHOLD - fabsf(chassisRotationDesiredWheelspeed)) /
                maxWheelSpeed,
            2.0f);

        rTranslationalGain = limitVal(rTranslationalGain, 0.0f, 1.0f);
    }
    return rTranslationalGain;
}

}  // namespace chassis

}  // namespace aruwsrc
