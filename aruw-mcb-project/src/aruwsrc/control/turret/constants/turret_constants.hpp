/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TURRET_CONSTANTS_HPP_
#define TURRET_CONSTANTS_HPP_

#include <stdint.h>

#include "aruwsrc/util_macros.hpp"

#if defined(ALL_STANDARDS)
#include "aruwsrc/robot/standard/standard_turret_constants.hpp"
#elif defined(TARGET_ENGINEER)
// necessary to satiate vision_coprocessor and world turret pid controller which relies on this
// header for number of turrets and gravity compensation values
// @todo use template parameter, normal parameter, or other workaround in the future
namespace aruwsrc::control::turret
{
static constexpr uint8_t NUM_TURRETS = 1;
static constexpr float TURRET_CG_X = 0.0f;
static constexpr float TURRET_CG_Z = 0.0f;
static constexpr float GRAVITY_COMPENSATION_SCALAR = 0.0f;
}  // namespace aruwsrc::control::turret
#elif defined(TARGET_HERO_PERSEUS)
#include "aruwsrc/robot/hero/hero_turret_constants.hpp"
#elif defined(TARGET_DRONE)
#include "aruwsrc/robot/drone/drone_turret_constants.hpp"
#elif defined(TARGET_SENTRY_HYDRA)
#include "aruwsrc/robot/sentry/sentry_turret_constants.hpp"
#elif defined(TARGET_DART)
#include "aruwsrc/robot/dart/dart_turret_constants.hpp"
#elif defined(TARGET_TESTBED)
#include "aruwsrc/robot/testbed/testbed_turret_constants.hpp"
#elif defined(TARGET_MOTOR_TESTER)
#include "aruwsrc/robot/motor_tester/motor_tester_turret_constants.hpp"
#endif

#endif  // TURRET_CONSTANTS_HPP_
