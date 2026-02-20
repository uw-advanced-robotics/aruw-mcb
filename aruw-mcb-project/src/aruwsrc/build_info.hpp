/*
 * Copyright (c) 2026-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef BUILD_INFO_HPP_
#define BUILD_INFO_HPP_

#define STRINGIFYMACRO(s) MACROSTR(s)
#define MACROSTR(s) #s

static constexpr char ROBOT_NAME[] = BUILD_TARGET;
static constexpr char LAST_USER[] = STRINGIFYMACRO(BUILD_USERNAME);
static constexpr char LAST_SHA[] = STRINGIFYMACRO(BUILD_SHA);
static constexpr char LAST_DATE[] = STRINGIFYMACRO(BUILD_DATE);
static constexpr char BRANCH_NAME[] = STRINGIFYMACRO(BUILD_BRANCH_NAME);

#endif  // BUILD_INFO_HPP_
