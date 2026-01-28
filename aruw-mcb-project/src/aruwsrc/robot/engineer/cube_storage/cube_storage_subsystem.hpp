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

#ifndef CUBE_STORAGE_SUBSYSTEM_HPP_
#define CUBE_STORAGE_SUBSYSTEM_HPP_

#include "aruwsrc/control/joint/homing/trigger_homed_joint_subsystem.hpp"
#include "aruwsrc/robot/engineer/digital_out_subsystem.hpp"

namespace aruwsrc::engineer::cube_storage
{
/**
 * Subsystem code for joints that don't need to be homed.
 */
class CubeStorageSubsystem : public aruwsrc::control::joint::homing::TriggerHomedJointSubsystem
{
public:

    CubeStorageSubsystem(tap::Drivers *drivers,
        tap::motor::MotorInterface &motor,
        aruwsrc::control::joint::homing::trigger::TriggerInterface &trigger,
        aruwsrc::control::joint::homing::TriggerHomedJointSubsystem::Config config);

    void initialize() override;

    void refresh() override;

    // can also be used as indices in arrary
    enum CubeOptions {
        LEFT = 0,
        RIGHT = 1,
        NONE = 2,
    }; 

    CubeOptions getCubeToRemove();
    CubeOptions getCubeToAdd();

    bool addCube(CubeOptions cubeToAdd);
    bool removeCube(CubeOptions cubeToRemove);

    bool storeWristPos(CubeOptions cubeToAdd, float wristPos);
    float getWristPos(CubeOptions cubeToRemove);

    void checkForCubes();

    float getPressure(CubeOptions cube);

protected:
    CubeOptions currentCube = CubeOptions::LEFT;
    bool hasCube[2] = {false, false};
    float wristPos[2] = {0.0f, 0.0f};
    //TODO: add vars for left and right pressure sensor
};
}  // namespace aruwsrc::engineer::cube_storage

#endif  // CUBE_STORAGE_SUBSYSTEM_HPP_