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

#ifndef TURRET_CONTROLLER_CONSTANTS_HPP_
#define TURRET_CONTROLLER_CONSTANTS_HPP_

namespace aruwsrc::control::turret
{
#if defined(ALL_SOLDIERS)

namespace world_rel_turret_imu
{
static constexpr float YAW_POS_P = 12.0f;
static constexpr float YAW_POS_I = 0.0f;
static constexpr float YAW_POS_D = 0.0f;
static constexpr float YAW_POS_MAX_ERROR_SUM = 60.0f;
static constexpr float YAW_POS_MAX_OUTPUT = 7000.0f;
static constexpr float YAW_POS_Q_DERIVATIVE_KALMAN = 1.0f;
static constexpr float YAW_POS_R_DERIVATIVE_KALMAN = 0.0f;
static constexpr float YAW_POS_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float YAW_POS_R_PROPORTIONAL_KALMAN = 0.0f;
static constexpr float YAW_POS_DEADZONE = 0.0f;

static constexpr float YAW_VEL_P = 280.0f;
static constexpr float YAW_VEL_I = 10.0f;
static constexpr float YAW_VEL_D = 0.0f;
static constexpr float YAW_VEL_MAX_ERROR_SUM = 10000.0f;
static constexpr float YAW_VEL_MAX_OUTPUT = 30000.0f;
static constexpr float YAW_VEL_Q_DERIVATIVE_KALMAN = 1.0f;
static constexpr float YAW_VEL_R_DERIVATIVE_KALMAN = 0.0f;
static constexpr float YAW_VEL_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float YAW_VEL_R_PROPORTIONAL_KALMAN = 0.5f;
static constexpr float YAW_VEL_DEADZONE = 0.0f;

static constexpr float PITCH_POS_P = 15.0f;
static constexpr float PITCH_POS_I = 0.0f;
static constexpr float PITCH_POS_D = 0.0f;
static constexpr float PITCH_POS_MAX_ERROR_SUM = 80.0f;
static constexpr float PITCH_POS_MAX_OUTPUT = 7000.0f;
static constexpr float PITCH_POS_Q_DERIVATIVE_KALMAN = 1.0f;
static constexpr float PITCH_POS_R_DERIVATIVE_KALMAN = 0.0f;
static constexpr float PITCH_POS_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float PITCH_POS_R_PROPORTIONAL_KALMAN = 0.0f;
static constexpr float PITCH_POS_DEADZONE = 0.0f;

static constexpr float PITCH_VEL_P = 280.0f;
static constexpr float PITCH_VEL_I = 8.0f;
static constexpr float PITCH_VEL_D = 0.0f;
static constexpr float PITCH_VEL_MAX_ERROR_SUM = 8000.0f;
static constexpr float PITCH_VEL_MAX_OUTPUT = 30000.0f;
static constexpr float PITCH_VEL_Q_DERIVATIVE_KALMAN = 1.0f;
static constexpr float PITCH_VEL_R_DERIVATIVE_KALMAN = 0.0f;
static constexpr float PITCH_VEL_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float PITCH_VEL_R_PROPORTIONAL_KALMAN = 0.5f;
static constexpr float PITCH_VEL_DEADZONE = 0.0f;
}  // namespace world_rel_turret_imu

namespace world_rel_chassis_imu
{
static constexpr float YAW_P = 3500.0f;
static constexpr float YAW_I = 0.0f;
static constexpr float YAW_D = 190.0f;
static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
static constexpr float YAW_MAX_OUTPUT = 32000.0f;
static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
static constexpr float YAW_R_DERIVATIVE_KALMAN = 40.0f;
static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float YAW_R_PROPORTIONAL_KALMAN = 0.0f;
}  // namespace world_rel_chassis_imu

namespace chassis_rel
{
static constexpr float YAW_P = 4000.0f;
static constexpr float YAW_I = 0.0f;
static constexpr float YAW_D = 190.0f;
static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
static constexpr float YAW_MAX_OUTPUT = 32000.0f;
static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
static constexpr float YAW_R_DERIVATIVE_KALMAN = 30.0f;
static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float YAW_R_PROPORTIONAL_KALMAN = 0.0f;

static constexpr float PITCH_P = 4000.0f;
static constexpr float PITCH_I = 0.0f;
static constexpr float PITCH_D = 130.0f;
static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
static constexpr float PITCH_MAX_OUTPUT = 32000.0f;
static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.0f;
static constexpr float PITCH_R_DERIVATIVE_KALMAN = 10.0f;
static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 2.0f;
}  // namespace chassis_rel

#elif defined(TARGET_HERO)

namespace world_rel_turret_imu
{
static constexpr float YAW_POS_P = 0.0f;
static constexpr float YAW_POS_I = 0.0f;
static constexpr float YAW_POS_D = 0.0f;
static constexpr float YAW_POS_MAX_ERROR_SUM = 0.0f;
static constexpr float YAW_POS_MAX_OUTPUT = 0.0f;
static constexpr float YAW_POS_Q_DERIVATIVE_KALMAN = 1.0f;
static constexpr float YAW_POS_R_DERIVATIVE_KALMAN = 0.0f;
static constexpr float YAW_POS_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float YAW_POS_R_PROPORTIONAL_KALMAN = 0.0f;
static constexpr float YAW_POS_DEADZONE = 0.0f;

static constexpr float YAW_VEL_P = 280.0f;
static constexpr float YAW_VEL_I = 10.0f;
static constexpr float YAW_VEL_D = 0.0f;
static constexpr float YAW_VEL_MAX_ERROR_SUM = 10000.0f;
static constexpr float YAW_VEL_MAX_OUTPUT = 30000.0f;
static constexpr float YAW_VEL_Q_DERIVATIVE_KALMAN = 1.0f;
static constexpr float YAW_VEL_R_DERIVATIVE_KALMAN = 0.0f;
static constexpr float YAW_VEL_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float YAW_VEL_R_PROPORTIONAL_KALMAN = 0.5f;
static constexpr float YAW_VEL_DEADZONE = 0.0f;

static constexpr float PITCH_POS_P = 0.0f;
static constexpr float PITCH_POS_I = 0.0f;
static constexpr float PITCH_POS_D = 0.0f;
static constexpr float PITCH_POS_MAX_ERROR_SUM = 0.0f;
static constexpr float PITCH_POS_MAX_OUTPUT = 0.0f;
static constexpr float PITCH_POS_Q_DERIVATIVE_KALMAN = 1.0f;
static constexpr float PITCH_POS_R_DERIVATIVE_KALMAN = 0.0f;
static constexpr float PITCH_POS_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float PITCH_POS_R_PROPORTIONAL_KALMAN = 0.0f;
static constexpr float PITCH_POS_DEADZONE = 0.0f;

static constexpr float PITCH_VEL_P = 0.0f;
static constexpr float PITCH_VEL_I = 0.0f;
static constexpr float PITCH_VEL_D = 0.0f;
static constexpr float PITCH_VEL_MAX_ERROR_SUM = 0.0f;
static constexpr float PITCH_VEL_MAX_OUTPUT = 0.0f;
static constexpr float PITCH_VEL_Q_DERIVATIVE_KALMAN = 1.0f;
static constexpr float PITCH_VEL_R_DERIVATIVE_KALMAN = 0.0f;
static constexpr float PITCH_VEL_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float PITCH_VEL_R_PROPORTIONAL_KALMAN = 0.0f;
static constexpr float PITCH_VEL_DEADZONE = 0.0f;
}  // namespace world_rel_turret_imu

namespace world_rel_chassis_imu
{
static constexpr float YAW_P = 0.0f;
static constexpr float YAW_I = 0.0f;
static constexpr float YAW_D = 0.0f;
static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
static constexpr float YAW_MAX_OUTPUT = 0.0f;
static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
static constexpr float YAW_R_DERIVATIVE_KALMAN = 0.0f;
static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float YAW_R_PROPORTIONAL_KALMAN = 0.0f;
}  // namespace world_rel_chassis_imu

namespace chassis_rel
{
static constexpr float YAW_P = 0.0f;
static constexpr float YAW_I = 0.0f;
static constexpr float YAW_D = 0.0f;
static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
static constexpr float YAW_MAX_OUTPUT = 0.0f;
static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
static constexpr float YAW_R_DERIVATIVE_KALMAN = 0.0f;
static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float YAW_R_PROPORTIONAL_KALMAN = 0.0f;

static constexpr float PITCH_P = 0.0f;
static constexpr float PITCH_I = 0.0f;
static constexpr float PITCH_D = 0.0f;
static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
static constexpr float PITCH_MAX_OUTPUT = 0.0f;
static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.0f;
static constexpr float PITCH_R_DERIVATIVE_KALMAN = 0.0f;
static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 0.0f;
}  // namespace chassis_rel

#elif defined(TARGET_SENTINEL)

namespace chassis_rel
{
static constexpr float YAW_P = 4000.0f;
static constexpr float YAW_I = 0.0f;
static constexpr float YAW_D = 130.0f;
static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
static constexpr float YAW_MAX_OUTPUT = 30000.0f;
static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
static constexpr float YAW_R_DERIVATIVE_KALMAN = 10.0f;
static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float YAW_R_PROPORTIONAL_KALMAN = 0.0f;

static constexpr float PITCH_P = 3400.0f;
static constexpr float PITCH_I = 0.0f;
static constexpr float PITCH_D = 100.0f;
static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
static constexpr float PITCH_MAX_OUTPUT = 30000.0f;
static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.0f;
static constexpr float PITCH_R_DERIVATIVE_KALMAN = 20.0f;
static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 0.0f;
}  // namespace chassis_rel

#endif
}  // namespace aruwsrc::control::turret

#endif  // TURRET_CONTROLLER_CONSTANTS_HPP_
