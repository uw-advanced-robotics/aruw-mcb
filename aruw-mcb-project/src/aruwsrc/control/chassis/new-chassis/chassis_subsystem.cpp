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

#include "chassis_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

#include "modm/math/matrix.hpp"

using namespace tap::algorithms;

namespace aruwsrc
{
namespace chassis
{
modm::Pair<int, float> ChassisSubsystem::lastComputedMaxWheelSpeed =
    CHASSIS_POWER_TO_MAX_SPEED_LUT[0];

ChassisSubsystem::ChassisSubsystem(
    tap::Drivers* drivers,
    std::vector<Wheel*>* wheels,
    tap::communication::sensors::current::CurrentSensorInterface* currentSensor)
    : tap::control::chassis::ChassisSubsystemInterface(drivers),
      wheels(*wheels),
      currentSensor(currentSensor),
      chasisSpeedRotationPID({
          AUTOROTATION_PID_KP,
          0.0f,
          AUTOROTATION_PID_KD,
          0.0f,
          AUTOROTATION_PID_MAX_OUTPUT,  // Able to take a lot of kalman stuff, deadzone, and floor
                                        // but might not need
      }),
      chassisPowerLimiter(
          drivers,
          currentSensor,
          STARTING_ENERGY_BUFFER,
          ENERGY_BUFFER_LIMIT_THRESHOLD,
          ENERGY_BUFFER_CRIT_THRESHOLD)
{
}

float ChassisSubsystem::chassisSpeedRotationPID(float currentAngleError, float errD)
{
    double currTime = tap::arch::clock::getTimeMicroseconds();
    double dt = currTime - prevTime;
    prevTime = currTime;
    return chasisSpeedRotationPID.runController(currentAngleError, errD, dt);
}

float ChassisSubsystem::calculateRotationTranslationalGain(float chassisRotationDesiredWheelspeed)
{
    // what we will multiply x and y speed by to take into account rotation
    float rTranslationalGain = 1.0f;

    // the x and y movement will be slowed by a fraction of auto rotation amount for maximizing
    // power consumption when the wheel rotation speed for chassis rotation is greater than the
    // MIN_ROTATION_THRESHOLD
    if (fabsf(chassisRotationDesiredWheelspeed) > MIN_ROTATION_THRESHOLD)
    {
        const float maxWheelSpeed = ChassisSubsystem::getMaxWheelSpeed(
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

void ChassisSubsystem::setDesiredOutput(float x, float y, float r)  //rpm, rpm, rpm
{
    float rotationTranslationGain = calculateRotationTranslationalGain(r);
    float tempMax = 0;
    float coeff;
    x = wheels[0]->rpmToMps(x);
    y = wheels[0]->rpmToMps(y);
    r = wheels[0]->rpmToMps(r) / HYPOTENUSE; //mps of chassis

    //TODO: make not magic number
    std::array<modm::Pair<float, float>, 4> desiredWheelVel;
    for (int i = 0; i < getNumChassisWheels(); i++)
    {
        desiredWheelVel[i] = wheels[i]->calculateDesiredWheelVelocity(
            rotationTranslationGain * x,  // scaled mps of raw motor
            rotationTranslationGain * y,  // scaled mps of raw motor
            r); //mps of chassis
        tempMax = std::max(tempMax, fabsf(desiredWheelVel[i].first));
    }
    for (int i = 0; i < getNumChassisWheels(); i++)
    {
        coeff = std::min(wheels[i]->config.maxWheelRPM / tempMax, 1.0f);
        wheels[i]->executeWheelVelocity(
            desiredWheelVel[i].first * coeff,
            desiredWheelVel[i].second);
    }
}


void ChassisSubsystem::initialize()
{
    for (int i = 0; i < getNumChassisWheels(); i++)
    {
        wheels[i]->initialize();
    }
}

void ChassisSubsystem::refresh()
{
    for (int i = 0; i < getNumChassisWheels(); i++)
    {
        wheels[i]->refresh();
    }
}

bool ChassisSubsystem::allMotorsOnline() const
{
    for (int i = 0; i < getNumChassisWheels(); i++)
    {
        if (!wheels[i]->allMotorsOnline())
        {
            return false;
        }
    }
    return true;
}

modm::Matrix<float, 3, 1> ChassisSubsystem::getActualVelocityChassisRelative() const
{
    modm::Matrix<float, 3, 1> wheelVelocity;
    return wheelVelocity.zeroMatrix();
}

}  // namespace chassis

}  // namespace aruwsrc
