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
#ifndef CUBE_POSITION_DIGITAL_OUT_COMMAND_HPP_
#define CUBE_POSITION_DIGITAL_OUT_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/control/digital/digital_out_subsystem.hpp"

#include "cube_storage_subsystem.hpp"

namespace aruwsrc::engineer::cube_storage
{
class CubePositionDigitalOutCommand : public tap::control::Command
{
public:
    CubePositionDigitalOutCommand(
        CubeStorageSubsystem& cubeStorage,
        DigitalOutSubsystem& leftSubsystem,
        DigitalOutSubsystem& rightSubsystem,
        const bool state)
        : cubeStorage(cubeStorage),
          leftSubsystem(leftSubsystem),
          rightSubsystem(rightSubsystem),
          state(state),
          running(false)
    {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(&leftSubsystem));
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(&rightSubsystem));
    }

    inline void initialize() override {}

    inline void execute() override
    {
        if (cubeStorage.getCurrentCube() == CubeStorageSubsystem::CubeOptions::LEFT)
        {
            leftSubsystem.set(state);
            running = true;
        }
        else if (cubeStorage.getCurrentCube() == CubeStorageSubsystem::CubeOptions::RIGHT)
        {
            rightSubsystem.set(state);
            running = true;
        }
        else
        {
            running = false;
        }
    }

    inline void end(bool) override
    {
        // would running still be true here?
        running = false;
    }

    inline bool isFinished() const override { return false; }

    const char* getName() const override { return "Cube Position Digital Output Command"; }

private:
    CubeStorageSubsystem& cubeStorage;
    DigitalOutSubsystem& leftSubsystem;
    DigitalOutSubsystem& rightSubsystem;
    const bool state;
    bool running;
};  // class CubePositionDigitalOutCommand

}  // namespace aruwsrc::engineer::cube_storage
#endif  // CUBE_POSITION_DIGITAL_OUT_COMMAND_HPP_
