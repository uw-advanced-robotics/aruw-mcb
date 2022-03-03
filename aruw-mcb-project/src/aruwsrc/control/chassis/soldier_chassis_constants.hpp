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

#ifndef SOLDIER_CHASSIS_CONSTANTS_HPP_
#define SOLDIER_CHASSIS_CONSTANTS_HPP_

#include "tap/communication/gpio/analog.hpp"

#include "modm/math/filter/pid.hpp"
#include "modm/math/interpolation/linear.hpp"

namespace aruwsrc::chassis
{
/**
 * Max wheel speed, measured in RPM of the encoder (rather than shaft)
 * we use this for wheel speed since this is how dji's motors measures motor speed.
 */
static constexpr int MIN_WHEEL_SPEED_SINGLE_MOTOR = 4000;
static constexpr int MAX_WHEEL_SPEED_SINGLE_MOTOR = 8000;
static constexpr int MIN_CHASSIS_POWER = 40;
static constexpr int MAX_CHASSIS_POWER = 120;

/**
 * The minimum desired wheel speed for chassis rotation, measured in RPM before
 * we start slowing down translational speed.
 */
static constexpr float MIN_ROTATION_THRESHOLD = 800.0f;

/**
 * Pin to use for current sensing
 */
static constexpr tap::gpio::Analog::Pin CURRENT_SENSOR_PIN = tap::gpio::Analog::Pin::S;

/// @see power_limiter.hpp for what these mean
static constexpr float STARTING_ENERGY_BUFFER = 60.0f;
static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 60.0f;
static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 10.0f;
static constexpr uint16_t POWER_CONSUMPTION_THRESHOLD = 20;
static constexpr float CURRENT_ALLOCATED_FOR_ENERGY_BUFFER_LIMITING = 30000;

static modm::Pid<float>::Parameter VELOCITY_PID_CONFIG{
    /** Kp */
    22.0f,
    /** Ki */
    0.2f,
    /** Kd */
    0.0f,
    /** maxErrorSum */
    5'000.0f,
    /**
     * This max output is measured in the c620 robomaster translated current.
     * Per the datasheet, the controllable current range is -16384 ~ 0 ~ 16384.
     * The corresponding speed controller output torque current range is
     * -20 ~ 0 ~ 20 A.
     */
    16'000.0f,
};

/**
 * Rotation PID: A PD controller for chassis autorotation.
 */
static constexpr float AUTOROTATION_PID_KP = 120.0f;
static constexpr float AUTOROTATION_PID_KD = 30.0f;
static constexpr float AUTOROTATION_PID_MAX_P = 5000.0f;
static constexpr float AUTOROTATION_PID_MAX_D = 5000.0f;
static constexpr float AUTOROTATION_PID_MAX_OUTPUT = 5500.0f;
static constexpr float AUTOROTATION_PID_TK = 0.0f;

/**
 * Radius of the wheels (m).
 */
static constexpr float WHEEL_RADIUS = 0.076;
/**
 * Distance from center of the two front wheels (m).
 */
static constexpr float WIDTH_BETWEEN_WHEELS_Y = 0.366f;
/**
 * Distance from center of the front and rear wheels (m).
 */
static constexpr float WIDTH_BETWEEN_WHEELS_X = 0.366f;
/**
 * Gimbal offset from the center of the chassis, see note above for explanation of x and y (m).
 */
static constexpr float GIMBAL_X_OFFSET = 0.0f;
/**
 * @see `GIMBAL_X_OFFSET`.
 */
static constexpr float GIMBAL_Y_OFFSET = 0.0f;
static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);

static constexpr float MAX_AUTOROTATE_DESIRED_WHEEL_SPEED_RPM = 500.0f;

/**
 * Maps particular max power thresholds to beyblade rotation thresholds.
 */
static constexpr modm::Pair<float, float> POWER_LIMIT_W_TO_ROTATION_TARGET_RPM_LUT[] =
    {{45, 3000}, {60, 3500}, {80, 5500}, {100, 6000}, {120, 7500}};

/**
 * Fraction betweeh [0, 1], what we multiply user translational input by when beyblading.
 */
static constexpr float BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER = 0.5f;

/**
 * Threshold, a fraction of the maximum translational speed that is used to determine if beyblade
 * speed should be reduced (when translating at an appreciable speed beyblade speed is reduced).
 */
static constexpr float BEYBLADE_TRANSLATIONAL_SPEED_THRESHOLD_FOR_ROTATION_SPEED_DECREASE = 0.5f;

/**
 * The fraction to cut rotation speed while moving and beyblading
 */
static constexpr float BEYBLADE_ROTATIONAL_SPEED_CUTOFF_WHEN_TRANSLATING = 0.5f;
/**
 * Rotational speed to update the beyblade ramp target by each iteration until final rotation
 * setpoint reached.
 */
static constexpr float BEYBLADE_RAMP_UPDATE_RMP = 0.125;
}  // namespace aruwsrc::chassis

#endif  // SOLDIER_CHASSIS_CONSTANTS_HPP_
