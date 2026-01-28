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
#ifndef CUBE_STORAGE_CHOOSE_REMOVE_COMMAND_HPP_
#define CUBE_STORAGE_CHOOSE_REMOVE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "cube_storage_subsystem.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_subsystem.hpp"

namespace aruwsrc::engineer::cube_storage

{
class CubeStorageChooseRemoveCommand : public tap::control::Command
{
public:
    CubeStorageChooseRemoveCommand(CubeStorageSubsystem &cubeStorage, aruwsrc::engineer::wrist::WristSubsystem &wristSubsystem);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char *getName() const override { return "Cube Storage Choose Remove Command"; }

private:
    CubeStorageSubsystem &cubeStorage;
    aruwsrc::engineer::wrist::WristSubsystem &wristSubsystem;

};  // class CubeStorageChooseRemoveCommand

}  // namespace aruwsrc::engineer::cube_storage
#endif  // CUBE_STORAGE_CHOOSE_REMOVE_COMMAND_HPP_
