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

#ifndef ENGINEER_TURRET_CONSTANTS_HPP_
#define ENGINEER_TURRET_CONSTANTS_HPP_

#include <cmath>

#include "tap/algorithms/fuzzy_pd.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"
#include "aruwsrc/control/turret/turret_motor_config.hpp"
#include "modm/container/pair.hpp"
#include "modm/math/geometry/angle.hpp"
#include "modm/math/interpolation/linear.hpp"

// Do not include this file directly: use turret_constants.hpp instead.
#ifndef TURRET_CONSTANTS_HPP_
#error "Do not include this file directly! Use turret_controller_constants.hpp instead."
#endif

namespace aruwsrc::control::turret
{
static constexpr uint8_t NUM_TURRETS = 1;

static constexpr float USER_YAW_INPUT_SCALAR = 0.01f;
static constexpr float USER_PITCH_INPUT_SCALAR = 0.02f;

static constexpr tap::can::CanBus CAN_BUS_YAW_MOTOR = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR5;

static constexpr tap::can::CanBus CAN_BUS_PITCH_MOTOR = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR6;

// turret pitch limits for different extension lengths, used in limit functions
// need to change
// if extension below threshold, we use the retracted limit which will limit the pitch more
// aggressively
inline constexpr float PITCH_UPPER_LIMIT_EXTENSION_RETRACTED = -0.6f;

// if extended far enough, we can pitch higher because the back of extension won't hit the
// chassis
inline constexpr float PITCH_UPPER_LIMIT_DEFAULT = -0.85f;

// if extension above threshold, limit the pitch so the extension doesnt hit the ground
inline constexpr float PITCH_LOWER_LIMIT_EXTENSION_EXTENDED = 0.5;
// lower limit if extension is retracted far enough
inline constexpr float PITCH_LOWER_LIMIT_DEFAULT = 0.5;

// if extension is below this value, limit the pitch more aggressively to prevent back of extension
// from hitting chassis
inline constexpr float MIN_EXTENSION_FOR_FULL_PITCH_UP = 0.2;

// if extension is above this value, we cannot allow full pitch down since extension will hit the
// ground
inline constexpr float MAX_EXTENSION_FOR_FULL_PITCH_DOWN = 0.42f;

// if we are not extended enough, limit pitch down so the back of extension doesnt hit the saturn
// ring
inline constexpr float MIN_EXTENSION_FOR_EXTRA_PITCH_DOWN = 0.3f;
inline constexpr float PITCH_DOWN_LIMIT_EXTENSION_PARTIAL = 0.12f;

// start limiting pitch upper range when the extension is beyond this value because before
// this point it's impossible to go over the height limit
inline constexpr float EXTENSION_THRESHOLD_FOR_PITCH_UPPER_LIMIT = 0.4f;

/**
 * Lookup table that maps extension position to pitch upper limit. In between points in the lookup
 * table, linear interpolation is used.
 */
inline constexpr modm::Pair<float, float> PITCH_UPPER_LIMIT_EXTENSION_TABLE[] = {
    {EXTENSION_THRESHOLD_FOR_PITCH_UPPER_LIMIT, PITCH_UPPER_LIMIT_DEFAULT},
    {0.47f, -0.67f},
    {0.53f, -0.63f},
    {0.63f, -0.57f},
    {0.73f, -0.52f},
};

static modm::interpolation::Linear<modm::Pair<float, float>>
    PITCH_UPPER_LIMIT_EXTENSION_INTERPOLATOR(
        PITCH_UPPER_LIMIT_EXTENSION_TABLE,
        MODM_ARRAY_SIZE(PITCH_UPPER_LIMIT_EXTENSION_TABLE));

inline float getPitchMinLimit(float extensionPosition)
{
    // TurretMotor expects numeric min/max radians. Pitch up is negative on engineer.
    if (extensionPosition < aruwsrc::control::turret::MIN_EXTENSION_FOR_FULL_PITCH_UP)
    {
        return aruwsrc::control::turret::PITCH_UPPER_LIMIT_EXTENSION_RETRACTED;
    }

    if (extensionPosition > EXTENSION_THRESHOLD_FOR_PITCH_UPPER_LIMIT)
    {
        return PITCH_UPPER_LIMIT_EXTENSION_INTERPOLATOR.interpolate(extensionPosition);
    }

    return aruwsrc::control::turret::PITCH_UPPER_LIMIT_DEFAULT;
}

inline float getPitchMaxLimit(float extensionPosition)
{
    // Pitch down is positive on engineer.
    if (extensionPosition < aruwsrc::control::turret::MIN_EXTENSION_FOR_EXTRA_PITCH_DOWN)
    {
        return aruwsrc::control::turret::PITCH_DOWN_LIMIT_EXTENSION_PARTIAL;
    }

    if (extensionPosition > aruwsrc::control::turret::MAX_EXTENSION_FOR_FULL_PITCH_DOWN)
    {
        return aruwsrc::control::turret::PITCH_LOWER_LIMIT_EXTENSION_EXTENDED;
    }
    return aruwsrc::control::turret::PITCH_LOWER_LIMIT_DEFAULT;
}

static constexpr aruwsrc::control::turret::TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 414,
    .minAngle = 0,        ///< Doesn't matter since yaw not limited
    .maxAngle = M_TWOPI,  ///< Doesn't matter since yaw not limited
    .limitMotorAngles = false,
};

inline constexpr float YAW_TURRET_GEAR_RATIO = 16.0f / 60.0f;

inline constexpr uint32_t PITCH_TURRET_ENCODER_HOME = 884;
inline constexpr float PITCH_TURRET_GEAR_RATIO = 1.0f / 8.0f;

static constexpr aruwsrc::control::turret::TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 0,  // NA since pitch motor encoder is incremental
    .minAngle = modm::toRadian(-43),
    .maxAngle = modm::toRadian(15),
    .limitMotorAngles = true,
};

static constexpr aruwsrc::control::turret::algorithms::TurretGravitationalForceOffset::
    TurretGravityParams TURRET_GRAVITY_CONFIG{
        .cgX = 0.0f,
        .cgZ = 0.0f,
        .gravityCompensatorMax = 0.0f};

inline constexpr float BINNED_ALIGNMENT_OFFSET = 0.32575804f;
inline constexpr float YAW_ALIGNMENT_OFFSET = 2.6f;

namespace world_rel_turret_imu
{
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 1.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 5.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 0,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 0.1f,
    .tRProportionalKalman = 0.4f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 2000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620 * (2.0f / 3.0f),
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_CONFIG = {
    .kp = 8.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 15.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 1.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_VEL_PID_CONFIG = {
    .kp = 5000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA * (1.0f / 2.0f),
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
}  // namespace world_rel_turret_imu

namespace world_rel_chassis_imu
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620,
    .tQDerivativeKalman = 0.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 0.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

}

namespace chassis_rel
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 35000.0f,
    .ki = 50000.0f,
    .kd = 4000.0f,
    .maxICumulative = 300.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 300.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.015f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 100000.0f,
    .ki = 30000.0f,
    .kd = 4000.0f,
    .maxICumulative = 1500.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 300.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

constexpr modm::Pair<float, float> LAMPREY_CALIBRATION_MAP[38] = {
    {0.000000f, 0.000000f}, {0.084791f, 0.084791f}, {0.144849f, 0.251115f}, {0.276070f, 0.446122f},
    {0.376597f, 0.625451f}, {0.420193f, 0.809689f}, {0.589557f, 0.983208f}, {0.595284f, 0.978966f},
    {0.764403f, 1.160585f}, {1.011761f, 1.340518f}, {1.222203f, 1.503940f}, {1.440547f, 1.698713f},
    {1.570195f, 1.868359f}, {1.719543f, 2.045803f}, {1.831457f, 2.234717f}, {2.007574f, 2.407335f},
    {2.206204f, 2.587151f}, {2.439671f, 2.762712f}, {2.728215f, 2.943489f}, {2.913296f, 3.119026f},
    {3.051125f, 3.298797f}, {3.180731f, 3.479441f}, {3.372349f, 3.675647f}, {3.605322f, 3.841017f},
    {3.871725f, 4.021602f}, {4.130415f, 4.213467f}, {4.367655f, 4.394318f}, {4.531702f, 4.569046f},
    {4.721763f, 4.738405f}, {4.863721f, 4.922260f}, {4.992419f, 5.104944f}, {5.151745f, 5.280353f},
    {5.186485f, 5.477458f}, {5.470463f, 5.649949f}, {5.589169f, 5.830747f}, {5.831504f, 6.008702f},
    {6.010724f, 6.185890f}, {6.283185f, 6.283185f}};

}  // namespace chassis_rel

}  // namespace aruwsrc::control::turret
#endif  // ENGINEER_TURRET_CONSTANTS_HPP_
