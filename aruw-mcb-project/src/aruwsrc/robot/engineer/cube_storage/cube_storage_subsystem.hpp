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

#include "tap/algorithms/transforms/transform.hpp"

#include "aruwsrc/control/joint/homing/trigger_homed_joint_subsystem.hpp"

using namespace tap::algorithms::transforms;

namespace aruwsrc::engineer::cube_storage
{
/**
 * Subsystem code for cube storage on 2026 engineer.
 */
class CubeStorageSubsystem : public aruwsrc::control::joint::homing::TriggerHomedJointSubsystem
{
public:
    CubeStorageSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface& motor,
        aruwsrc::control::joint::homing::trigger::TriggerInterface& trigger,
        aruwsrc::control::joint::homing::TriggerHomedJointSubsystem::Config config);

    void initialize() override;

    // can also be used as indices in array
    enum CubeOptions
    {
        LEFT = 0,
        RIGHT = 1,
        ERROR = 2,
    };

    CubeOptions getCubeToRemove();
    CubeOptions getCubeToAdd();
    CubeOptions getCurrentCube();

    /**
     * sets the setpoint based on the current cube position
     * @return true if set sucessfully, false otherwise
     */
    bool setSetpointToCurrentCube();

    /** Tell subsystem that you have added a cube
     * @param CubeOptions which cube you are adding
     * @return true for success, false for failure
     */
    bool addCube();

    /** Tell subsystem that you have removed a cube
     * @return true for success, false for failure
     */
    bool removeCube();

    bool storeActiveCubeStoreToCube(Transform cubeStoreToCube);

    /**
     * @return wrist position for current cube position as a transform
     * if no current cube will return an identity transform
     */
    Transform getActiveCubeStoreToCube();

    std::array<bool, 2> checkForCubes();

    float getPressure(CubeOptions cube);
    bool isReady();

protected:
    CubeOptions currentCube;
    bool hasCube[2];
    Transform cubeStoresToCube[2];
    // TODO: add vars for left and right pressure sensor
};
}  // namespace aruwsrc::engineer::cube_storage

#endif  // CUBE_STORAGE_SUBSYSTEM_HPP_