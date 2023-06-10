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

#ifndef SENTRY_BEEHIVE_TURRET_CONSTANTS_HPP_
#define SENTRY_BEEHIVE_TURRET_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/turret/turret_motor_config.hpp"
#include "modm/math/geometry/angle.hpp"
#include "modm/math/geometry/vector3.hpp"

// Do not include this file directly: use turret_constants.hpp instead.
#ifndef TURRET_CONSTANTS_HPP_
#error "Do not include this file directly! Use turret_controller_constants.hpp instead."
#endif

namespace aruwsrc::control::turret
{

static constexpr uint8_t NUM_TURRETS = 2;

static constexpr float MAJOR_USER_YAW_INPUT_SCALAR = 0.007f;

static constexpr float MINOR_USER_YAW_INPUT_SCALAR = 0.02f;
static constexpr float MINOR_USER_PITCH_INPUT_SCALAR = 0.02f;

static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR5;

static constexpr float TURRET_CG_X = 9.14f;
static constexpr float TURRET_CG_Z = 9.45f;
static constexpr float GRAVITY_COMPENSATION_SCALAR = 8'000.0f;

// The distance from turret 0 to turret 1 in meters
// @TODO: deprecate ??
static modm::Vector3f OFFSET_TURRET_0_TO_TURRET_1 = modm::Vector3f(-0.17511f, -.27905f, 0.0f);
static constexpr float PITCH_YAW_OFFSET = 0.045f;

namespace turretMajor
{
static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 8115,
    .minAngle = 0,
    .maxAngle = M_TWOPI,
    .limitMotorAngles = false,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 20.0f,
    .ki = 000.0f,
    .kd = -1500.0f,
    .maxICumulative = 3'000.0f,
    .maxOutput = 500.0f,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 3000.0f,
    .ki = 1.0f,
    .kd = -37000.0f,
    .maxICumulative = 500.0f,
    .maxOutput = 25'000.0f,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

// Turret Major has a double DJI motor, so we need to have two CAN Buses
static constexpr tap::can::CanBus CAN_BUS_MOTOR_1 = tap::can::CanBus::CAN_BUS1;
static constexpr tap::can::CanBus CAN_BUS_MOTOR_2 = tap::can::CanBus::CAN_BUS2;
// static constexpr boolean majorInverted = true;
}  // namespace turretMajor

namespace girlboss
{

static constexpr uint8_t turretID = 0;
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 1345,
    .minAngle = modm::toRadian(-30),
    .maxAngle = modm::toRadian(210),
    .limitMotorAngles = true,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 1900,
    .minAngle = modm::toRadian(-9),
    .maxAngle = modm::toRadian(42),
    .limitMotorAngles = true,
};

static constexpr float majorToTurretR = 0.145;
static constexpr float default_launch_speed = 14.0f;
static constexpr tap::communication::serial::RefSerial::Rx::MechanismID barrelID =
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1;
}  // namespace girlboss

namespace malewife
{
static constexpr uint8_t turretID = 1;
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 6842,
    .minAngle = modm::toRadian(-210),
    .maxAngle = modm::toRadian(30),
    .limitMotorAngles = true,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 3285,
    .minAngle = modm::toRadian(-9),
    .maxAngle = modm::toRadian(42),
    .limitMotorAngles = true,
};
static constexpr float majorToTurretR = -0.145;
static constexpr float default_launch_speed = 14.0f;
static constexpr tap::communication::serial::RefSerial::Rx::MechanismID barrelID =
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_2;
}  // namespace malewife

namespace major_rel
{

namespace malewife
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 38'000.0f,
    .ki = 370.0f,
    .kd = 3'600.0f,
    .maxICumulative = 2'000.0f,
    .maxOutput = 28'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 50'000.0f,
    .ki = 75.0f,
    .kd = 4'000.0f,
    .maxICumulative = 3'000.0f,
    .maxOutput = 28'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 20.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};
}  // namespace malewife

namespace girlboss
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 38'000.0f,
    .ki = 370.0f,
    .kd = 3'600.0f,
    .maxICumulative = 2'000.0f,
    .maxOutput = 28'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 50'000.0f,
    .ki = 75.0f,
    .kd = 4'000.0f,
    .maxICumulative = 3'000.0f,
    .maxOutput = 28'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 20.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};
}  // namespace girlboss
}  // namespace major_rel

}  // namespace  aruwsrc::control::turret

#endif  // SENTRY_BEEHIVE_TURRET_CONSTANTS_HPP_
