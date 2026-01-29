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
#include "tap/algorithms/transforms/transform.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"
#include "aruwsrc/control/turret/turret_motor_config.hpp"
#include "aruwsrc/robot/sentry/turret/sentry_turret_minor_subsystem.hpp"  // for turretID enum (could go somewhere else)
#include "modm/math/geometry/angle.hpp"
#include "modm/math/geometry/vector3.hpp"

// Do not include this file directly: use turret_constants.hpp instead.
#ifndef TURRET_CONSTANTS_HPP_
#error "Do not include this file directly! Use turret_constants.hpp instead."
#endif

namespace aruwsrc::control::turret
{
#ifdef TARGET_SENTRY_NAME
static constexpr uint8_t NUM_TURRETS = 1;
#else
static constexpr uint8_t NUM_TURRETS = 2;
#endif

static constexpr float MAJOR_USER_YAW_INPUT_SCALAR = 0.007f;

static constexpr float MINOR_USER_YAW_INPUT_SCALAR = 0.008f;
static constexpr float MINOR_USER_PITCH_INPUT_SCALAR = 0.008f;

static constexpr float TORQUE_TO_DESIRED_OUT =
    1.3f / tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA;  // 1.3Nm max torque
static constexpr float TURRET_WEIGHT_KG = 1.44730f;     // From CAD

static constexpr algorithms::TurretGravitationalForceOffset::TurretGravityParams
    TURRET_GRAVITY_CONFIG{
        .cgX = -18.24f,
        .cgZ = 20.35f,
        .gravityCompensatorMax = 5118.6f,
    };

static constexpr float TURRET_MINOR_OFFSET = 0.14222f;

static constexpr SmoothPidConfig IMU_SYNC_PID_CONFIG = {
    .kp = 0.042f,
    .ki = 0,
    .kd = 0,
    .maxICumulative = M_PI,
    .maxOutput = M_PI,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

namespace turretMajor
{
static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 1060,
    .minAngle = 0,
    .maxAngle = M_TWOPI,
    .limitMotorAngles = false,
};

static const tap::algorithms::transforms::Transform TURRET_MAJOR_IMU_MOUNTING_TRANSFORM(
    0,
    0,
    0,
    M_PI,
    0,
    0);

namespace chassisFrameController
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 120'000.0f,
    .ki = 500.0f,
    .kd = 2000.0f,
    .maxICumulative = 8'000.0f,
    .maxOutput = static_cast<uint16_t>(tap::motor::DjiMotor::MAX_OUTPUT_C620 * 0.6),
    .tRDerivativeKalman = 100.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

}  // namespace chassisFrameController

namespace worldFrameCascadeController
{
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 9.0f,
    .ki = 0.03f,
    .kd = 0.2f,
    .maxICumulative = 0.2f,
    .maxOutput = 6.0f,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 10'000.0f,
    .ki = 0.0f,
    .kd = 4'000.0f,
    .maxICumulative = 1'500.0f,
    .maxOutput = static_cast<uint16_t>(tap::motor::DjiMotor::MAX_OUTPUT_C620 * 0.6),
    .tRDerivativeKalman = 60'000.0f,  // Gain needs to be so high for the motors to actually do
                                      // anything that motor encoder resolution becomes a problem
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 60.0f,
    .errDeadzone = 0.0f,
};

}  // namespace worldFrameCascadeController

// Turret Major has a double DJI motor, so we need to have two CAN Buses
static constexpr tap::can::CanBus CAN_BUS_MOTOR = tap::can::CanBus::CAN_BUS1;

static constexpr float MAX_VEL_ERROR_INPUT = 20.0f;
static constexpr float TURRET_MINOR_TORQUE_RATIO = 0.0f;

static constexpr float FEEDFORWARD_GAIN = 0.0f;
}  // namespace turretMajor

static constexpr float ANGLES_OF_FREEDOM = modm::toRadian(255.7f);
static constexpr float PADDING = modm::toRadian(5);

#ifdef TARGET_SENTRY_NAME
// Single turret minor for SENTINEL 2026 - named "widow"
namespace turretWidow
{
static constexpr uint8_t turretID = 0;

static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;

static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR5;

static constexpr float CENTER_OF_FREEDOM = modm::toRadian(90);

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 3142,
    .minAngle = CENTER_OF_FREEDOM - ANGLES_OF_FREEDOM / 2.f + PADDING,
    .maxAngle = CENTER_OF_FREEDOM + ANGLES_OF_FREEDOM / 2.f - PADDING,
    .limitMotorAngles = true,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 1364,
    .minAngle = modm::toRadian(-18),  // actual CAD limit -20
    .maxAngle = modm::toRadian(33),   // actual CAD limit is 35
    .limitMotorAngles = true,
};

static constexpr float majorToTurretR = 0.145;
static constexpr float DEFAULT_LAUNCH_SPEED = 25.0f;
static constexpr tap::communication::serial::RefSerial::Rx::MechanismID barrelID =
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_2;
}  // namespace turretWidow

#else  // TARGET_SENTRY_ECLIPSE

namespace turretLeft
{
static constexpr uint8_t turretID = 0;

static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;

static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR5;

static constexpr float CENTER_OF_FREEDOM = modm::toRadian(90);

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 3142,
    .minAngle = CENTER_OF_FREEDOM - ANGLES_OF_FREEDOM / 2.f + PADDING,
    .maxAngle = CENTER_OF_FREEDOM + ANGLES_OF_FREEDOM / 2.f - PADDING,
    .limitMotorAngles = true,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 1364,
    .minAngle = modm::toRadian(-18),  // actual CAD limit -20
    .maxAngle = modm::toRadian(33),   // actual CAD limit is 35
    .limitMotorAngles = true,
};

static constexpr float majorToTurretR = 0.145;
static constexpr float DEFAULT_LAUNCH_SPEED = 25.0f;
static constexpr tap::communication::serial::RefSerial::Rx::MechanismID barrelID =
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_2;
}  // namespace turretLeft

namespace turretRight
{
static constexpr uint8_t turretID = 1;

static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR5;

static constexpr float CENTER_OF_FREEDOM = modm::toRadian(-90);

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 5088,
    .minAngle = CENTER_OF_FREEDOM - ANGLES_OF_FREEDOM / 2.f + PADDING,
    .maxAngle = CENTER_OF_FREEDOM + ANGLES_OF_FREEDOM / 2.f - PADDING,
    .limitMotorAngles = true,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 5433,
    .minAngle = modm::toRadian(-18),  // actual CAD limit -20
    .maxAngle = modm::toRadian(33),   // actual CAD limit is 35
    .limitMotorAngles = true,
};
static constexpr float majorToTurretR = -0.145;
static constexpr float DEFAULT_LAUNCH_SPEED = 20.0f;
static constexpr tap::communication::serial::RefSerial::Rx::MechanismID barrelID =
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1;

}  // namespace turretRight

#endif  // TARGET_SENTRY_NAME vs TARGET_SENTRY_ECLIPSE

namespace minorPidConfigs
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG_CHASSIS_FRAME = {
    .kp = 100'000.0f,
    .ki = 300.0f,
    .kd = 7'000.0f,
    .maxICumulative = 2'000.0f,
    .maxOutput = 15'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 80.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG_CHASSIS_FRAME = {
    .kp = 120'000.0f,
    .ki = 12.0f,
    .kd = 6'800.0f,
    .maxICumulative = 4'000.0f,
    .maxOutput = 15'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 60.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig LEFT_YAW_PID_CONFIG_WORLD_FRAME_VEL = {
    .kp = 5'300.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig RIGHT_YAW_PID_CONFIG_WORLD_FRAME_VEL = {
    .kp = 5'000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG_WORLD_FRAME_POS = {
    .kp = 29.0f,
    .ki = 0.18f,
    .kd = 0.0f,
    .maxICumulative = 1.0f,
    .maxOutput = 12.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG_WORLD_FRAME_VEL = {
    .kp = 6'000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG_WORLD_FRAME_POS = {
    .kp = 34.0f,
    .ki = 0.2f,
    .kd = 0.0f,
    .maxICumulative = 0.05f,
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
