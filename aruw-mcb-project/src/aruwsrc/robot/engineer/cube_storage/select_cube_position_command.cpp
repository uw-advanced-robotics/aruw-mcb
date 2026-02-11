/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "select_cube_position_command.hpp"

#include "cube_storage_subsystem.hpp"

namespace aruwsrc::engineer::cube_storage
{
SelectCubePositionCommand::SelectCubePositionCommand(  // two const references
    CubeStorageSubsystem &cubeStorage,
    aruwsrc::control::joint::JointSubsystem &jointSubsystem,
    bool addCube
    // TurretToCubeTransform &turretToCubeTransformer,
    // TurretToSuctionTransform &turretToSuctionTransformer
    )
    : cubeStorage(cubeStorage),
      jointSubsystem(jointSubsystem)
// turretToCubeTransformer(turretToCubeTransformer),
// turretToSuctionTransformer(turretToSectionTransformer)
{
}

void SelectCubePositionCommand::initialize()
{
    if (addCube)
    {
        cubeStorage.getCubeToAdd();
        cubeStorage.storeWristPos(Transform(0, 0, 0, 0, 0, 0));  // TODO: update; 3 motors on wrist
        // use transforms systems
    }
    else
    {
        cubeStorage.getCubeToRemove();
        // jointSubsystem.setSetpoint(cubeStorage.getWristPos());
        // ^ make smth in jointsubsystem accept a transform, oliver problem i think?
    }

    // multiply transformers here & give position to wrist
}

void SelectCubePositionCommand::execute() {}

void SelectCubePositionCommand::end(bool) {}

bool SelectCubePositionCommand::isFinished() const { return true; }
}  // namespace aruwsrc::engineer::cube_storage
