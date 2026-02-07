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

#include "aruwsrc/robot/engineer/cube_storage/cube_storage_choose_add_command.hpp"

namespace aruwsrc::engineer::cube_storage
{
CubeStorageChooseAddCommand::CubeStorageChooseAddCommand(  // two const references
    CubeStorageSubsystem &cubeStorage,
    aruwsrc::control::joint::JointSubsystem &jointSubsystem
    // TurretToCubeTransform &turretToCubeTransformer,
    // TurretToSuctionTransform &turretToSuctionTransformer
    )
    : cubeStorage(cubeStorage),
      jointSubsystem(jointSubsystem)
// turretToCubeTransformer(turretToCubeTransformer),
// turretToSuctionTransformer(turretToSectionTransformer)
{
}

void CubeStorageChooseAddCommand::initialize()
{
    cubeStorage.getCubeToAdd();
    cubeStorage.storeWristPos(nullptr);  // TODO: update; 3 motors on wrist
    // use transforms systems

    // multiply transformers here & give position to wrist
}

void CubeStorageChooseAddCommand::execute() {}

void CubeStorageChooseAddCommand::end(bool) {}

bool CubeStorageChooseAddCommand::isFinished() const { return true; }
}  // namespace aruwsrc::engineer::cube_storage
