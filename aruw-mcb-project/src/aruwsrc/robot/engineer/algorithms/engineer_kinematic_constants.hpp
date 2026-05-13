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

#ifndef ENGINEER_KINEMATIC_CONSTANTS_HPP_
#define ENGINEER_KINEMATIC_CONSTANTS_HPP_

#include "tap/algorithms/transforms/position.hpp"
#include "tap/algorithms/transforms/transform.hpp"

#include "aruwsrc/algorithms/point_mass.hpp"

namespace aruwsrc::engineer::algorithms
{
static const tap::algorithms::transforms::Position CHASSIS_TO_TURRET_YAW_POS(0, 0, 0.22064);
static const tap::algorithms::transforms::Position TURRET_YAW_TO_TURRET_PITCH_POS(
    -0.025,
    0,
    0.16192);
static const tap::algorithms::transforms::Position TURRET_PITCH_TO_EXTENSION_ZERO_POS(
    0.23,
    0,
    0.1);  // TODO
static const tap::algorithms::transforms::Transform WRIST_TO_END_EFFECTOR(0.0803, 0, 0, 0, 0, 0);
static const tap::algorithms::transforms::Transform EXTENSION_TO_VTM_GIMBAL(
    0,
    0,
    0,
    0,
    0,
    0);  // TODO
static const tap::algorithms::transforms::Transform TURRET_YAW_TO_CUBE_STORE_FRAME(
    -0.03089,
    0,
    0.19657,
    0,
    modm::toRadian(35.6),
    0);
static const tap::algorithms::transforms::Transform TURRET_YAW_TO_REALSENSE(
    -0.10106,
    0.0875,
    0.28883,
    0,
    modm::toRadian(-25),
    0);
static const tap::algorithms::transforms::Transform CUBE_STORE_FRAME_TO_CUBE_DIST(0, 0, 0, 0, 0, 0);
inline constexpr float CUBE_STORE_RADIUS = 0.27100;
inline constexpr float CUBE_STORE_RADIUS_ANGLE = modm::toRadian(72);
static const tap::algorithms::transforms::Transform CUBE_STORE_CENTER_TO_CUBE_STORE_1(
    CUBE_STORE_RADIUS* cosf(CUBE_STORE_RADIUS_ANGLE),
    CUBE_STORE_RADIUS* sinf(CUBE_STORE_RADIUS_ANGLE),
    0.04343,
    0,
    modm::toRadian(-80),
    CUBE_STORE_RADIUS_ANGLE);
static const tap::algorithms::transforms::Transform CUBE_STORE_CENTER_TO_CUBE_STORE_2(
    CUBE_STORE_RADIUS* cosf(-CUBE_STORE_RADIUS_ANGLE),
    CUBE_STORE_RADIUS* sinf(-CUBE_STORE_RADIUS_ANGLE),
    0.04343,
    0,
    modm::toRadian(-80),
    -CUBE_STORE_RADIUS_ANGLE);

static const tap::algorithms::transforms::Transform END_EFFECTOR_TO_WRIST =
    WRIST_TO_END_EFFECTOR.getInverse();
static const tap::algorithms::transforms::Transform VTM_GIMBAL_TO_EXTENSION =
    EXTENSION_TO_VTM_GIMBAL.getInverse();
static const tap::algorithms::transforms::Transform CUBE_STORE_1_TO_CUBE_STORE_CENTER =
    CUBE_STORE_CENTER_TO_CUBE_STORE_1.getInverse();
static const tap::algorithms::transforms::Transform CUBE_STORE_2_TO_CUBE_STORE_CENTER =
    CUBE_STORE_CENTER_TO_CUBE_STORE_2.getInverse();
static const tap::algorithms::transforms::Transform TURRET_YAW_TO_CUBE_DIST =
    TURRET_YAW_TO_CUBE_STORE_FRAME.composeStatic(CUBE_STORE_FRAME_TO_CUBE_DIST);

// To simplify kinematic calculations, we usually treat the wrist roll joint as if it were also
// located in the differential. This is how far along its axis it's actually physically located
// static constexpr float TRUE_WRIST_ROLL_OFFSET = 0;

// Center of Masses
static const aruwsrc::algorithms::PointMass MASS_BEYOND_WRIST{
    .mass = 0.471,
    .location =
        tap::algorithms::transforms::Position(0.0457, -0.01134, 0)};  // TODO may have to negate y
static constexpr float EXTENSION_STATIONARY_MASS = 1;                 // TODO
static constexpr float EXTENSION_MIDDLE_MASS = 1;                     // TODO
static constexpr float EXTENSION_END_MASS = 1;                        // TODO
static const aruwsrc::algorithms::PointMass MASS_BETWEEN_TURRET_PITCH_AND_WRIST_ZERO_EXT{
    .mass = EXTENSION_STATIONARY_MASS + EXTENSION_MIDDLE_MASS + EXTENSION_END_MASS,
    .location = tap::algorithms::transforms::Position(0, 0, 0)};  // TODO
static constexpr float EXT_TO_COM_POS_BETWEEN_TURRET_PITCH_AND_WRIST_SCALAR =
    (EXTENSION_MIDDLE_MASS / 2 + EXTENSION_END_MASS) /
    (EXTENSION_STATIONARY_MASS + EXTENSION_MIDDLE_MASS + EXTENSION_END_MASS);

}  // namespace aruwsrc::engineer::algorithms
#endif  // ENGINEER_KINEMATIC_CONSTANTS_HPP_
