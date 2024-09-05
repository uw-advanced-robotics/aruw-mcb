/*
* Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef GO_TO_POSITION_SUPERSTRUCTURE_COMMAND_HPP_
#define GO_TO_POSITION_SUPERSTRUCTURE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "arm_superstructure.hpp"

namespace aruwsrc::engineer::arm
{

struct Position
{
    float lift;
    float reach;
    float yaw;
    float pitch;
    float roll;
};

class GoToPositionSuperstructure : public tap::control::Command
{
public: 
    GoToPositionSuperstructure(ArmSuperstructure* superstructure, Position position);

    void initialize() override;

    void execute() override {}

    void end(bool interrupted) override {}

    bool isReady() override {
        return superstructure->isOnline();
    }

    bool isFinished() const override {
        return superstructure->atSetpoint();
    }

    const char* getName() const override { return "Go to position superstructure"; }

private:
    ArmSuperstructure* superstructure;
    Position position;
};  // class GoToPositionSuperstructure

}  // aruwsrc::engineer::arm
#endif  // GO_TO_POSITION_SUPERSTRUCTURE_COMMAND_HPP_
