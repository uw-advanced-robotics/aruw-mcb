/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "cube_storage_subsystem.hpp"

using namespace aruwsrc::control::joint::homing;
using namespace aruwsrc::control::joint::homing::trigger;

namespace aruwsrc::engineer::cube_storage
{
CubeStorageSubsystem::CubeStorageSubsystem(
    tap::Drivers *drivers,
    tap::motor::MotorInterface &motor,
    TriggerInterface &trigger,
    Config config)
    : Subsystem(drivers),
    TriggerHomedJointSubsystem(drivers, motor, trigger, config)
{
}

void CubeStorageSubsystem::initialize()
{
    TriggerHomedJointSubsystem::initialize();
    currentCube = CubeOptions::LEFT;
    hasCube[CubeOptions::LEFT] = false;
    hasCube[CubeOptions::RIGHT] = false;
}

void CubeStorageSubsystem::refresh() { TriggerHomedJointSubsystem::refresh(); }

CubeStorageSubsystem::CubeOptions CubeStorageSubsystem::getCubeToAdd()
{
    checkForCubes();
    if (!hasCube[CubeOptions::LEFT])
    {
        currentCube = CubeOptions::LEFT;
    }
    else if (!hasCube[CubeOptions::RIGHT])
    {
        currentCube = CubeOptions::RIGHT;
    }
    else
    {
        currentCube = CubeOptions::NONE;
    }
    return currentCube;
}

CubeStorageSubsystem::CubeOptions CubeStorageSubsystem::getCurrentCube() { return currentCube; }

CubeStorageSubsystem::CubeOptions CubeStorageSubsystem::getCubeToRemove()
{
    checkForCubes();
    if (hasCube[CubeOptions::LEFT])
    {
        return CubeOptions::LEFT;
    }
    else if (hasCube[CubeOptions::RIGHT])
    {
        return CubeOptions::RIGHT;
    }
    else
    {
        return CubeOptions::NONE;
    }
}

/* tell subsystem that you have added a cube
 * @param CubeOptions which cube you are adding
 * @return true for success, false for failure
 */
bool CubeStorageSubsystem::addCube()
{
    if (currentCube != CubeOptions::NONE)
    {
        hasCube[currentCube] = 1;
        return true;
    }
    return false;
}

/* tell subsystem that you have added a cube
 * @param CubeOptions which cube you are adding
 * @return true for success, false for failure
 */
bool CubeStorageSubsystem::removeCube()
{
    if (currentCube != CubeOptions::NONE)
    {
        hasCube[currentCube] = 0;
        return true;
    }
    return false;
}

bool CubeStorageSubsystem::storeWristPos(float newWristPosition)
{
    if (currentCube != CubeOptions::NONE)
    {
        if (!hasCube[currentCube])
        {
            wristPos[currentCube] = newWristPosition;
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

float CubeStorageSubsystem::getWristPos()
{
    if (currentCube != CubeOptions::NONE)
    {
        if (hasCube[currentCube])
        {
            return wristPos[currentCube];
        }
        else
        {
            return std::numeric_limits<float>::quiet_NaN();
        }
    }
    return std::numeric_limits<float>::quiet_NaN();
}

void CubeStorageSubsystem::checkForCubes()
{
    hasCube[CubeOptions::LEFT] = getPressure(CubeOptions::LEFT) > 1.0f;  // TODO: update this
    hasCube[CubeOptions::RIGHT] = getPressure(CubeOptions::RIGHT) > 1.0f;
}

float CubeStorageSubsystem::getPressure(CubeOptions cube)
{
    if (cube != CubeOptions::NONE)
    {
        return 1.0f;  // TODO: update to read the pressure
    }
    return std::numeric_limits<float>::quiet_NaN();
}

}  // namespace aruwsrc::engineer::cube_storage