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

#ifndef FRAMES_HPP_
#define FRAMES_HPP_

namespace aruwsrc::algorithms::state
{

#if defined(ALL_SENTRIES)
enum class Frame
{
    WORLD,
    CHASSIS,
    TURRET_MAJOR,
    TURRET
};
#elif defined(TARGET_ENGINEER)
enum class Frame
{
    WORLD,
    CHASSIS,
    GANTRY,
    WRIST,
    END_EFFECTOR,
    TURRET
};
#else
enum class Frame
{
    WORLD,
    CHASSIS,
    TURRET
};
#endif

}  // namespace aruwsrc::algorithms::state

#endif  // FRAMES_HPP_
