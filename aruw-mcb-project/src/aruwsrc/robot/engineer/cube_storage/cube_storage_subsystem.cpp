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
    : TriggerHomedJointSubsystem(drivers, motor, trigger, config), Subsystem(drivers)
{
}

void CubeStorageSubsystem::initialize() {
    TriggerHomedJointSubsystem::initialize();
    currentCube = CubeOptions::LEFT;
    hasCube[CubeOptions::LEFT] = false;
    hasCube[CubeOptions::RIGHT] = false;
}

void CubeStorageSubsystem::refresh()
{   
    TriggerHomedJointSubsystem::refresh();
}

CubeStorageSubsystem::CubeOptions CubeStorageSubsystem::getCubeToAdd() {
    checkForCubes();
    if (!hasCube[CubeOptions::LEFT]) {
        return CubeOptions::LEFT;
    } else if (!hasCube[CubeOptions::RIGHT]) {
        return CubeOptions::RIGHT;
    } else {
        return CubeOptions::NONE;
    }
}

CubeStorageSubsystem::CubeOptions CubeStorageSubsystem::getCubeToAdd() {
    checkForCubes();
    if (hasCube[CubeOptions::LEFT]) {
        return CubeOptions::LEFT;
    } else if (hasCube[CubeOptions::RIGHT]) {
        return CubeOptions::RIGHT;
    } else {
        return CubeOptions::NONE;
    }
}

/* tell subsystem that you have added a cube
* @param CubeOptions which cube you are adding
* @return true for success, false for failure
*/
bool CubeStorageSubsystem::addCube(CubeOptions cubeToAdd) {
    if (cubeToAdd != CubeOptions::NONE) {
        hasCube[cubeToAdd] = 1;
        return true;
    }
    return false;
}

/* tell subsystem that you have added a cube
* @param CubeOptions which cube you are adding
* @return true for success, false for failure
*/
bool CubeStorageSubsystem::removeCube(CubeOptions cubeToRemove) {
    if (cubeToRemove != CubeOptions::NONE) {
        hasCube[cubeToRemove] = 0;
        return true;
    }
    return false;
}

bool CubeStorageSubsystem::storeWristPos(CubeOptions cubeToAdd, float newWristPosition) {
    if (cubeToAdd != CubeOptions::NONE) {
        if (!hasCube[cubeToAdd]) {
            wristPos[cubeToAdd] = newWristPosition;
            return true;
        } else {
            return false;
        }
    }
    return false;
}

float CubeStorageSubsystem::getWristPos(CubeOptions cubeToRemove) {
    if (cubeToRemove != CubeOptions::NONE) {
        if (hasCube[cubeToRemove]) {
            return wristPos[cubeToRemove];
        } else {
            return std::numeric_limits<float>::quiet_NaN();
        }
    }
    return std::numeric_limits<float>::quiet_NaN();
}

void CubeStorageSubsystem::checkForCubes() {
    hasCube[CubeOptions::LEFT] = getPressure(CubeOptions::LEFT) > 1.0f; //TODO: update this
    hasCube[CubeOptions::RIGHT] = getPressure(CubeOptions::RIGHT) > 1.0f;

}

float CubeStorageSubsystem::getPressure(CubeOptions cube) {
    if (cube != CubeOptions::NONE) {
        return 1.0f; //TODO: update to read the pressure
    }
    return std::numeric_limits<float>::quiet_NaN();
}

}  // namespace aruwsrc::engineer::cube_storage