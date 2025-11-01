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
#ifndef SCORE_POSITION_COMMAND_HPP_
#define SCORE_POSITION_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/control/joint/joint_subsystem.hpp"
#include "aruwsrc/robot/engineer/engineer_setpoint_constants.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_subsystem.hpp"

using namespace aruwsrc::engineer::wrist;

namespace aruwsrc::engineer
{
class ScorePositionCommand : public tap::control::Command
{
public:
    ScorePositionCommand(
        aruwsrc::control::joint::JointSubsystem &gantryLift,
        WristSubsystem &wrist,
        aruwsrc::control::joint::JointSubsystem &roll);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char *getName() const override { return "Score Position Command"; }

    void cyclePositions(ScorePositions scorePos);

private:
    aruwsrc::control::joint::JointSubsystem &gantryLift;
    WristSubsystem &wrist;
    aruwsrc::control::joint::JointSubsystem &roll;
    ScorePositions scoringPosition;

};  // class ScorePositionCommand

}  // namespace aruwsrc::engineer
#endif  // SCORE_POSITION_COMMAND_HPP_