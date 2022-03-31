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

#ifndef SENTINEL_CONSTANTS_HPP_
#define SENTINEL_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/motor/dji_motor.hpp"

#include "modm/container/pair.hpp"

namespace aruwsrc::control::turret
{
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR5;

static constexpr float YAW_START_ANGLE = 90.0f;
static constexpr float YAW_MIN_ANGLE = 5.0f;
static constexpr float YAW_MAX_ANGLE = 175.0f;
static constexpr float PITCH_START_ANGLE = 90.0f;
static constexpr float PITCH_MIN_ANGLE = 80.0f;
static constexpr float PITCH_MAX_ANGLE = 165.0f;

static constexpr uint16_t YAW_START_ENCODER_POSITION = 2801;
static constexpr uint16_t PITCH_START_ENCODER_POSITION = 4035;

static constexpr float TURRET_CG_X = 0;
static constexpr float TURRET_CG_Z = 0;
static constexpr float GRAVITY_COMPENSATION_SCALAR = 1.0f;

namespace chassis_rel
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 4000.0f,
    .ki = 0.0f,
    .kd = 130.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 30000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 10.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 3400.0f,
    .ki = 0.0f,
    .kd = 100.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 30000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 20.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};
}  // namespace chassis_rel
}  // namespace  aruwsrc::control::turret

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

// Do not include this file directly, use agitator_consants.hpp

namespace aruwsrc::control::agitator::constants
{
static constexpr tap::algorithms::SmoothPidConfig AGITATOR_PID_CONFIG =
    {.kp = 300'000.0f, .ki = 0.0f, .kd = 50.0f, .maxICumulative = 0.0f, .maxOutput = 16000.0f};

static constexpr tap::motor::MotorId AGITATOR_MOTOR_ID = tap::motor::MOTOR7;
static constexpr tap::can::CanBus AGITATOR_MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS1;
}  // namespace aruwsrc::control::agitator::constants

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

#endif  // SENTINEL_CONSTANTS_HPP_
