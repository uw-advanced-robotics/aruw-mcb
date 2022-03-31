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

#ifndef SOLDIER_CONSTANTS_HPP_
#define SOLDIER_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/communication/gpio/analog.hpp"
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/motor/dji_motor.hpp"

#include "modm/math/filter/pid.hpp"
#include "modm/math/geometry/angle.hpp"
#include "modm/math/interpolation/linear.hpp"

namespace aruwsrc::control::turret
{
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR5;

static constexpr float YAW_START_ANGLE = 90.0f;
static constexpr float PITCH_START_ANGLE = 90.0f;
static constexpr float YAW_MIN_ANGLE = 0.0f;
static constexpr float YAW_MAX_ANGLE = 180.0f;

#ifdef TARGET_SOLDIER_2021
static constexpr float PITCH_MIN_ANGLE = 40.0f;
static constexpr float PITCH_MAX_ANGLE = 117.0f;
static constexpr uint16_t YAW_START_ENCODER_POSITION = 6821;
static constexpr uint16_t PITCH_START_ENCODER_POSITION = 7500;

static constexpr float TURRET_CG_X = 0;
static constexpr float TURRET_CG_Z = 0;
static constexpr float GRAVITY_COMPENSATION_SCALAR = 0.0f;
#else
static constexpr float PITCH_MIN_ANGLE = 65.0f;
static constexpr float PITCH_MAX_ANGLE = 117.0f;
static constexpr uint16_t YAW_START_ENCODER_POSITION = 1100;
static constexpr uint16_t PITCH_START_ENCODER_POSITION = 7500;

static constexpr float TURRET_CG_X = 0;
static constexpr float TURRET_CG_Z = 0;
static constexpr float GRAVITY_COMPENSATION_SCALAR = 0;
#endif

namespace world_rel_turret_imu
{
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 22.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 10'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};  // namespace world_rel_turret_imu

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
#if defined(TARGET_SOLDIER_2021)
    .kp = 550.0f,
#else
    .kp = 700.0f,
#endif
    .ki = 5.0f,
    .kd = 0.0f,
    .maxICumulative = 2'000.0f,
    .maxOutput = 30000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_CONFIG = {
    .kp = 22.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 10'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_VEL_PID_CONFIG = {
    .kp = 280.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 30000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
};
}  // namespace world_rel_turret_imu

namespace world_rel_chassis_imu
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 3500.0f,
    .ki = 0.0f,
    .kd = 190.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 32000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};
}  // namespace world_rel_chassis_imu

namespace chassis_rel
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 4000.0f,
    .ki = 0.0f,
    .kd = 190.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 32000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 30.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 4000.0f,
    .ki = 0.0f,
    .kd = 130.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 32000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 10.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 2.0f,
    .errDeadzone = 0.0f,
};
}  // namespace chassis_rel
}  // namespace aruwsrc::control::turret

namespace aruwsrc::can
{
// Come back to this - may be helpful to have other constants in here
static constexpr tap::can::CanBus TURRET_MCB_CAN_BUS = tap::can::CanBus::CAN_BUS1;
}  // namespace aruwsrc::can

namespace aruwsrc::serial
{
static constexpr tap::communication::serial::Uart::UartPort VISION_COPROCESSOR_TX_UART_PORT =
    tap::communication::serial::Uart::UartPort::Uart2;

static constexpr tap::communication::serial::Uart::UartPort VISION_COPROCESSOR_RX_UART_PORT =
    tap::communication::serial::Uart::UartPort::Uart3;
}  // namespace aruwsrc::serial

namespace aruwsrc::control::agitator::constants
{
// position PID terms
// PID terms for soldier
static constexpr tap::algorithms::SmoothPidConfig AGITATOR_PID_CONFIG =
    {.kp = 200'000.0f, .ki = 0.0f, .kd = 100.0f, .maxICumulative = 0.0f, .maxOutput = 16000.0f};

static constexpr tap::motor::MotorId AGITATOR_MOTOR_ID = tap::motor::MOTOR7;
static constexpr tap::can::CanBus AGITATOR_MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS1;
static constexpr bool isAgitatorInverted = false;

/**
 * The jamming constants. Agitator is considered jammed if difference between setpoint
 * and current angle is > `JAMMING_DISTANCE` radians for >= `JAMMING_TIME` ms;
 *
 * @warning: `JAMMING_DISTANCE` must be less than the smallest movement command
 *
 * This should be positive or else weird behavior can occur
 */
static constexpr float AGITATOR_JAMMING_DISTANCE = M_PI / 20;
static constexpr uint32_t JAMMING_TIME = 70;
}  // namespace aruwsrc::control::agitator::constants

namespace aruwsrc::chassis
{
/**
 * Maps max power (in Watts) to max chassis wheel speed (RPM).
 */
static constexpr modm::Pair<int, float> CHASSIS_POWER_TO_MAX_SPEED_LUT[] =
    {{50, 4'500}, {60, 5'700}, {70, 6'400}, {80, 6'700}, {100, 7'000}, {120, 8'000}};

static modm::interpolation::Linear<modm::Pair<int, float>> CHASSIS_POWER_TO_SPEED_INTERPOLATOR(
    CHASSIS_POWER_TO_MAX_SPEED_LUT,
    MODM_ARRAY_SIZE(CHASSIS_POWER_TO_MAX_SPEED_LUT));

/**
 * The minimum desired wheel speed for chassis rotation when translational scaling via
 * calculateRotationTranslationalGain is performed.
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
static constexpr float AUTOROTATION_PID_KP = 100.0f;
static constexpr float AUTOROTATION_PID_KD = 1.0f;
static constexpr float AUTOROTATION_PID_MAX_P = 4'000.0f;
static constexpr float AUTOROTATION_PID_MAX_D = 5'000.0f;
static constexpr float AUTOROTATION_PID_MAX_OUTPUT = 5'500.0f;
static constexpr float AUTOROTATION_MIN_SMOOTHING_ALPHA = 0.001f;

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

/**
 * Fraction of max chassis speed that will be applied to rotation when beyblading
 */
static constexpr float BEYBLADE_ROTATIONAL_SPEED_FRACTION_OF_MAX = 0.75f;

/**
 * Fraction between [0, 1], what we multiply user translational input by when beyblading.
 */
static constexpr float BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER = 0.6f;

/**
 * Threshold, a fraction of the maximum translational speed that is used to determine if beyblade
 * speed should be reduced (when translating at an appreciable speed beyblade speed is reduced).
 */
static constexpr float
    BEYBLADE_TRANSLATIONAL_SPEED_THRESHOLD_MULTIPLIER_FOR_ROTATION_SPEED_DECREASE = 0.5f;

/**
 * The fraction to cut rotation speed while moving and beyblading
 */
static constexpr float BEYBLADE_ROTATIONAL_SPEED_MULTIPLIER_WHEN_TRANSLATING = 0.7f;
/**
 * Rotational speed to update the beyblade ramp target by each iteration until final rotation
 * setpoint reached, in RPM.
 */
static constexpr float BEYBLADE_RAMP_UPDATE_RAMP = 50;
}  // namespace aruwsrc::chassis

namespace aruwsrc::control::launcher
{
static constexpr modm::Pair<float, float> LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[] =
    {{0.0f, 0.0f}, {15.0f, 4600.0f}, {18.0f, 5000.0f}, {30.0f, 7200.2f}, {32.0f, 8300.0f}};
}  // namespace aruwsrc::control::launcher

namespace aruwsrc::control::control_operator_interface
{
/**
 * Max acceleration in rpm/s^2 of the chassis in the x direction
 */
static constexpr float MAX_ACCELERATION_X = 10'000.0f;
static constexpr float MAX_DECELERATION_X = 20'000.0f;

/**
 * Max acceleration in rpm/s^2 of the chassis in the y direction
 */
static constexpr float MAX_ACCELERATION_Y = 9'000.0f;
static constexpr float MAX_DECELERATION_Y = 20'000.0f;

/**
 * Max acceleration in rpm/s^2 of the chassis in the r direction
 */
static constexpr float MAX_ACCELERATION_R = 40'000.0f;
static constexpr float MAX_DECELERATION_R = 50'000.0f;
}  // namespace aruwsrc::control::control_operator_interface

#endif  // SOLDIER_CONSTANTS_HPP_
