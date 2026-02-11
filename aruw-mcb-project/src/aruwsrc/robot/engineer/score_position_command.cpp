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

#include "score_position_command.hpp"

namespace aruwsrc::engineer
{
ScorePositionCommand::ScorePositionCommand(
    aruwsrc::control::joint::JointSubsystem& gantryLift,
    WristSubsystem& wrist,
    aruwsrc::control::joint::JointSubsystem& roll)
    : gantryLift(gantryLift),
      wrist(wrist),
      roll(roll)
{
    addSubsystemRequirement(&gantryLift);
    addSubsystemRequirement(&wrist);
    addSubsystemRequirement(&roll);
};

void ScorePositionCommand::cyclePositions(ScorePositions scorePos) { scoringPosition = scorePos; }

void ScorePositionCommand::initialize()
{
    float gantryLiftSetpoint;
    float wristYawSetpoint;
    float wristPitchSetpoint;
    float wristRollSetpoint;

    if (scoringPosition == ScorePositions::one)
    {
        gantryLiftSetpoint = gantryLiftScoreOneSetpoint;
        wristYawSetpoint = wristYawScoreOneSetpoint;
        wristPitchSetpoint = wristPitchScoreOneSetpoint;
        wristRollSetpoint = wristRollScoreOneSetpoint;
    }
    else if (scoringPosition == ScorePositions::two)
    {
        gantryLiftSetpoint = gantryLiftScoreTwoSetpoint;
        wristYawSetpoint = wristYawScoreTwoSetpoint;
        wristPitchSetpoint = wristPitchScoreTwoSetpoint;
        wristRollSetpoint = wristRollScoreTwoSetpoint;
    }
    else
    {
        gantryLiftSetpoint = gantryLiftScoreThreeSetpoint;
        wristYawSetpoint = wristYawScoreThreeSetpoint;
        wristPitchSetpoint = wristPitchScoreThreeSetpoint;
        wristRollSetpoint = wristRollScoreThreeSetpoint;
    }

    gantryLift.setSetpoint(gantryLiftSetpoint);
    wrist.setSetpointTheta1(wristYawSetpoint);    // theta1 is azimuth/yaw
    wrist.setSetpointTheta2(wristPitchSetpoint);  // theta2 is pitch
    roll.setSetpoint(wristRollSetpoint);
}

void ScorePositionCommand::execute() {}

void ScorePositionCommand::end(bool) {}

bool ScorePositionCommand::isFinished() const
{
    return gantryLift.atSetpoint() && wrist.atSetpoint() && roll.atSetpoint();
}
}  // namespace aruwsrc::engineer