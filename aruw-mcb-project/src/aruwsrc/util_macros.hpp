/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef UTIL_MACROS_HPP_
#define UTIL_MACROS_HPP_

/**
 * Define a helper macro that makes it easier to specify at compile time something that should be
 * true for all standards.
 */
#if defined(TARGET_STANDARD_ELSA) || defined(TARGET_STANDARD_SPIDER)
#define ALL_STANDARDS
#endif

/**
 * A helper macro that makes it easier to specify at compile time something that should be true for
 * all sentries.
 */
#if defined(TARGET_SENTRY_HYDRA)
#define ALL_SENTRIES
#endif

#endif  // UTIL_MACROS_HPP_
