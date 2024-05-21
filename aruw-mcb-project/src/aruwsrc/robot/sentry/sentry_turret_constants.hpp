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

#ifndef SENTRY_TURRET_CONSTANTS_HPP_
#define SENTRY_TURRET_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/turret/turret_motor_config.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_minor_subsystem.hpp"  // for turretID enum (could go somewhere else)
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

static constexpr float TURRET_CG_X = 32.5f;
static constexpr float TURRET_CG_Z = 32.3f;
static constexpr float GRAVITY_COMPENSATION_SCALAR =
    -13'000.0f;  // Right turret is -14'000 for some reason

// The distance from turret 0 to turret 1 in meters
static modm::Vector3f OFFSET_TURRET_0_TO_TURRET_1 = modm::Vector3f(-0.17511f, -.27905f, 0.0f);
static constexpr float PITCH_YAW_OFFSET = 0.045f;

static constexpr float TURRET_MINOR_OFFSET = 0.145f;

namespace turretMajor
{
static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 1228,
    .minAngle = 0,
    .maxAngle = M_TWOPI,
    .limitMotorAngles = false,
};

namespace chassisFrameController
{
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 10000,
    .ki = 0.0002f,
    .kd = -93.0f,
    .maxICumulative = 5.0f,
    .maxOutput = 500.0f,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

}  // namespace chassisFrameController

namespace worldFrameCascadeController
{
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 5.3f,
    .ki = 0.0002f,
    .kd = -143.0f,
    .maxICumulative = 5.0f,
    .maxOutput = 500.0f,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 18'500.0f,
    .ki = 500.0f,
    .kd = -580'000.0f,
    .maxICumulative = 1'500.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020,
    .tRDerivativeKalman = 90'000.0f,  // Gain needs to be so high for the motors to actually do
                                      // anything that motor encoder resolution becomes a problem
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 600.0f,
    .errDeadzone = 0.0f,
};

}  // namespace worldFrameCascadeController

// Turret Major has a double DJI motor, so we need to have two CAN Buses
static constexpr tap::can::CanBus CAN_BUS_MOTOR_1 = tap::can::CanBus::CAN_BUS1;
static constexpr tap::can::CanBus CAN_BUS_MOTOR_2 = tap::can::CanBus::CAN_BUS2;
// static constexpr boolean majorInverted = true;

static constexpr float MAX_VEL_ERROR_INPUT = 20.0f;
static constexpr float TURRET_MINOR_TORQUE_RATIO = 0.0f;

static constexpr float FEEDFORWARD_GAIN = 0.0f;
}  // namespace turretMajor

namespace turretLeft
{
static constexpr uint8_t turretID = 0;

static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;

static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR5;

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 7510,
    .minAngle = modm::toRadian(-15),
    .maxAngle = modm::toRadian(195),
    .limitMotorAngles = true,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 4780,
    .minAngle = modm::toRadian(-13),
    .maxAngle = modm::toRadian(35),
    .limitMotorAngles = true,
};

static constexpr float majorToTurretR = 0.145;
static constexpr float default_launch_speed = 14.0f;
static constexpr tap::communication::serial::RefSerial::Rx::MechanismID barrelID =
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1;
}  // namespace turretLeft

namespace turretRight
{
static constexpr uint8_t turretID = 1;

static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR5;

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 2107,
    .minAngle = modm::toRadian(-195),  // actual CAD limit is -200
    .maxAngle = modm::toRadian(15),    // actual CAD limit is -20
    .limitMotorAngles = true,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 4727,
    .minAngle = modm::toRadian(-13),  // actual CAD limit is -15
    .maxAngle = modm::toRadian(35),   // actual CAD limit 37
    .limitMotorAngles = true,
};
static constexpr float majorToTurretR = -0.145;
static constexpr float default_launch_speed = 14.0f;
static constexpr tap::communication::serial::RefSerial::Rx::MechanismID barrelID =
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_2;

}  // namespace turretRight

namespace minorPidConfigs
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG_CHASSIS_FRAME = {
    .kp = 80'000.0f,
    .ki = 100.0f,
    .kd = 6'000.0f,
    .maxICumulative = 2'000.0f,
    .maxOutput = 28'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG_CHASSIS_FRAME = {
    .kp = 90'000.0f,
    .ki = 100.0f,
    .kd = 6'000.0f,
    .maxICumulative = 3'000.0f,
    .maxOutput = 28'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 20.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG_WORLD_FRAME_VEL = {
    .kp = 13'500.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 28'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG_WORLD_FRAME_POS = {
    .kp = 25.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 5.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG_WORLD_FRAME_VEL = {
    .kp = 12'000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 28'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG_WORLD_FRAME_POS = {
    .kp = 25.0f,
    .ki = 1.0f,
    .kd = 0.0f,
    .maxICumulative = 0.1f,
    .maxOutput = 5.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};
}  // namespace minorPidConfigs
}  // namespace  aruwsrc::control::turret

#endif  // SENTRY_TURRET_CONSTANTS_HPP_
