/*
 * Copyright (c) 2026-2027 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ENGINEER_TRAJECTORY_IK_CONSTANTS_HPP_
#define ENGINEER_TRAJECTORY_IK_CONSTANTS_HPP_

namespace aruwsrc::engineer
{
static constexpr float PER_POSE_TIME = 1.5f;
static constexpr float TEST_EXTENSION_MIN_M = 0.02f;
static constexpr float TEST_EXTENSION_NEUTRAL_M = 0.25f;
static constexpr float TEST_EXTENSION_MAX_M = 0.60f;
}  // namespace aruwsrc::engineer

#endif  // ENGINEER_TRAJECTORY_IK_CONSTANTS_HPP_
