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

#include "half_swerve_chassis_subsystem.hpp"

using namespace tap::algorithms;

using namespace aruwsrc::chassis;

HalfSwerveChassisSubsystem::HalfSwerveChassisSubsystem(
    tap::Drivers* drivers,
    tap::communication::sensors::current::CurrentSensorInterface* currentSensor,
    Module* moduleOne,
    Module* moduleTwo,
    aruwsrc::virtualMCB::VirtualDjiMotor* parallelEncoder,
    aruwsrc::virtualMCB::VirtualDjiMotor* perpendiculoluarEncoder,
    const float forwardMatrixArray[24])
    : HolonomicChassisSubsystem(drivers, currentSensor),
      modules{moduleOne, moduleTwo},
      parallelEncoder(parallelEncoder),
      perpendiculoluarEncoder(perpendiculoluarEncoder),
      forwardMatrix(forwardMatrixArray)
{
}

void HalfSwerveChassisSubsystem::initialize()
{
    for (unsigned int i = 0; i < NUM_MODULES; i++)
    {
        modules[i]->initialize();
    }
}

bool HalfSwerveChassisSubsystem::allMotorsOnline() const
{
    bool online = true;
    for (unsigned int i = 0; i < NUM_MODULES; i++) online &= modules[i]->allMotorsOnline();
    return online;
}

void HalfSwerveChassisSubsystem::setZeroRPM()
{
    for (unsigned int i = 0; i < NUM_MODULES; i++) modules[i]->setZeroRPM();
}

Module* HalfSwerveChassisSubsystem::getModule(unsigned int i)
{
    if (i >= NUM_MODULES) return nullptr;
    return modules[i];
}

void HalfSwerveChassisSubsystem::setDesiredOutput(float x, float y, float r)
{
    x = modules[0]->wheel.rpmToMps(x);           // convert input from motor rpm to m/s
    y = modules[0]->wheel.rpmToMps(y);           // convert input from motor rpm to m/s
    r = modules[0]->wheel.rpmToMps(r) / 0.205f;  // convert input from motor rpm to rad/s
    // TODO: REPLACE WITH CONSTANT FROM CONSTANTS FILE
    //^simplified tank drive rotation calculation that doesnt take width_y into account
    swerveDriveCalculate(
        x,
        y,
        r,
        getMaxWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit));
}

void HalfSwerveChassisSubsystem::swerveDriveCalculate(float x, float y, float r, float maxWheelRPM)
{
    desiredRotation = modules[LF]->wheel.mpsToRpm(r) * 0.205f;
    float maxInitialSpeed = 0;
    for (unsigned int i = 0; i < NUM_MODULES; i++)
    {
        desiredModuleSpeeds[i][0] = modules[i]->calculate(x, y, r);
        maxInitialSpeed = std::max(maxInitialSpeed, desiredModuleSpeeds[i][0]);
    }

    float scaleCoeff = std::min(maxWheelRPM / maxInitialSpeed, 1.0f);

    for (unsigned int i = 0; i < NUM_MODULES; i++)
    {
        modules[i]->scaleAndSetDesiredState(scaleCoeff);
    }
}
// int wtf = 0, wtf2 = 0;
void HalfSwerveChassisSubsystem::refresh()
{
    for (unsigned int i = 0; i < NUM_MODULES; i++)
    {
        modules[i]->refresh();
    }
    // wtf++;
    limitChassisPower();
}
float powerLimitFrac;
void HalfSwerveChassisSubsystem::limitChassisPower()
{
    // use power limiting object to compute initial power limiting fraction
    // wtf2++;
    currentSensor->update();
    powerLimitFrac = chassisPowerLimiter.getPowerLimitRatio();

    // short circuit if power limiting doesn't need to be applied
    if (compareFloatClose(1.0f, powerLimitFrac, 1E-3))
    {
        return;
    }

    for (unsigned int i = 0; i < NUM_MODULES; i++)
    {
        modules[i]->limitPower(powerLimitFrac);
    }
}

modm::Matrix<float, 3, 1> HalfSwerveChassisSubsystem::getActualVelocityChassisRelative() const
{
    modm::Matrix<float, 6, 1> actualModuleVectors;
    for (unsigned int i = 0; i < NUM_MODULES; i++)
    {
        modm::Matrix<float, 2, 1> moduleVel = modules[i]->getActualModuleVelocity();
        actualModuleVectors[2 * i][0] = moduleVel[0][0];
        actualModuleVectors[2 * i + 1][0] = moduleVel[1][0];
    }
    actualModuleVectors[4][0] = getParallelMotorVelocity();
    actualModuleVectors[5][0] = getPerpendicularMotorVelocity();

    return forwardMatrix * actualModuleVectors;
}

void HalfSwerveChassisSubsystem::setObservableState()
{
    modm::Matrix<float, 6, 1> actualModuleVectors;
    for (unsigned int i = 0; i < NUM_MODULES; i++)
    {
        modm::Matrix<float, 2, 1> moduleVel = modules[i]->getActualModuleVelocity();
        actualModuleVectors[2 * i][0] = moduleVel[0][0];
        actualModuleVectors[2 * i + 1][0] = moduleVel[1][0];
    }
    actualModuleVectors[4][0] = getParallelMotorVelocity();
    actualModuleVectors[5][0] = getPerpendicularMotorVelocity();
    moduleVels.setX1(actualModuleVectors[0][0]);
    moduleVels.setY1(actualModuleVectors[1][0]);
    moduleVels.setO1(actualModuleVectors[4][0]);
    moduleVels.setX2(actualModuleVectors[2][0]);
    moduleVels.setY2(actualModuleVectors[3][0]);
    moduleVels.setO2(actualModuleVectors[5][0]);
}

modm::Matrix<float, 3, 1> HalfSwerveChassisSubsystem::getDesiredVelocityChassisRelative() const
{
    modm::Matrix<float, 6, 1> desiredModuleVectors;
    for (unsigned int i = 0; i < NUM_MODULES; i++)
    {
        modm::Matrix<float, 2, 1> moduleVel = modules[i]->getDesiredModuleVelocity();
        desiredModuleVectors[2 * i][0] = moduleVel[0][0];
        desiredModuleVectors[2 * i + 1][0] = moduleVel[1][0];
    }
    desiredModuleVectors[4][0] = 0;
    desiredModuleVectors[5][0] = 0;
    return forwardMatrix * desiredModuleVectors;
}