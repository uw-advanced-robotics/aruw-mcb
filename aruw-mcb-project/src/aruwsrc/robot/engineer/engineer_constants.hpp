/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ENGINEER_CONSTANTS_HPP_
#define ENGINEER_CONSTANTS_HPP_

namespace aruwsrc::engineer
{
// fyi, some positions/constants should be here but are in the the individual subsystem constants bc
// :(((((
enum ScorePositions
{
    one,
    two,
    three
};

// score positions
// while the code for this exists, none of it been tested lol
static constexpr float gantryLiftScoreOneSetpoint = 1;  // TODO: find positions
static constexpr float wristYawScoreOneSetpoint = 1;
static constexpr float wristPitchScoreOneSetpoint = 1;
static constexpr float wristRollScoreOneSetpoint = 1;

static constexpr float gantryLiftScoreTwoSetpoint = 1;  // TODO: find positions
static constexpr float wristYawScoreTwoSetpoint = 1;
static constexpr float wristPitchScoreTwoSetpoint = 1;
static constexpr float wristRollScoreTwoSetpoint = 1;

static constexpr float gantryLiftScoreThreeSetpoint = 1;  // TODO: find positions
static constexpr float wristYawScoreThreeSetpoint = 1;
static constexpr float wristPitchScoreThreeSetpoint = 1;
static constexpr float wristRollScoreThreeSetpoint = 1;
}  // namespace aruwsrc::engineer
#endif  // ENGINEER_CONSTANTS_HPP_