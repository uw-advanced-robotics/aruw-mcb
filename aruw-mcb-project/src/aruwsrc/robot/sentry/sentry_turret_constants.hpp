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
#include "tap/communication/sensors/encoder/can_encoder/can_encoder.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"
#include "aruwsrc/control/turret/algorithms/turret_spring_compensation.hpp"
#include "aruwsrc/control/turret/algorithms/turret_stos_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_stos_turret_controller.hpp"
#include "aruwsrc/control/turret/turret_motor_config.hpp"
#include "aruwsrc/robot/sentry/turret/sentry_turret_minor_subsystem.hpp"  // for turretID enum (could go somewhere else)
#include "modm/container/pair.hpp"
#include "modm/math/geometry/angle.hpp"
#include "modm/math/geometry/vector3.hpp"

// Do not include this file directly: use turret_constants.hpp instead.
#ifndef TURRET_CONSTANTS_HPP_
#error "Do not include this file directly! Use turret_constants.hpp instead."
#endif

namespace aruwsrc::control::turret
{
static constexpr uint8_t NUM_TURRETS = 1;

static constexpr float MAJOR_USER_YAW_INPUT_SCALAR = 0.007f;

static constexpr float MINOR_USER_YAW_INPUT_SCALAR = 0.008f;
static constexpr float MINOR_USER_PITCH_INPUT_SCALAR = 0.008f;

static constexpr float DESIRED_OUT_TO_TORQUE =
    1.3f / tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA;  // 1.3Nm max torque
static constexpr float TURRET_WEIGHT_KG = 1.44730f;     // From CAD

static constexpr algorithms::TurretGravitationalForceOffset::TurretGravityParams
    TURRET_GRAVITY_CONFIG{
        .cgX = 42.45f,
        .cgZ = -71.51f,
        .gravityCompensatorMax = -14277.7f,
    };
static constexpr algorithms::TurretSpringForceOffset::TurretSpringParams TURRET_SPRING_CONFIG{
    .turretPitchMountX = -25.0f,
    .turretPitchMountZ = 0.0f,
    .turretYawMountX = -24.4f,
    .turretYawMountZ = -70.5f,
    .springConstant = -10.0f,
    .springFreeLength = 55.0f,
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
inline constexpr float PULLEY_RATIO =
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508 * (30.0f / 95.0f);

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 1060,
    .minAngle = 0,
    .maxAngle = M_TWOPI,
    .limitMotorAngles = false,
};

static constexpr tap::can::CanBus YAW_LAMPREY_CAN_BUS = tap::can::CanBus::CAN_BUS2;
static constexpr tap::encoder::CanEncoderId YAW_LAMPREY_CAN_ID = tap::encoder::CanEncoderId::ID7;
static constexpr bool YAW_LAMPREY_INVERTED = false;

inline constexpr float BINNED_ALIGNMENT_OFFSET = 3.712f;
inline constexpr float SENTRY_YAW_ALIGNMENT_OFFSET = modm::toRadian(267.0f);

// inline const modm::Pair<float, float> LAMPREY_CALIBRATION_MAP[0] = {};
inline constexpr modm::Pair<float, float> LAMPREY_CALIBRATION_MAP[38] = {
    {0.000000f, 0.000000f}, {0.075346f, 0.075346f}, {0.218188f, 0.253815f}, {0.404305f, 0.435452f},
    {0.548394f, 0.613726f}, {0.733642f, 0.793966f}, {0.897285f, 0.974016f}, {1.051091f, 1.151980f},
    {1.175210f, 1.331009f}, {1.362350f, 1.511143f}, {1.431943f, 1.690569f}, {1.575330f, 1.869899f},
    {1.723736f, 2.050683f}, {1.836452f, 2.229278f}, {1.985133f, 2.408738f}, {2.129243f, 2.587978f},
    {2.273207f, 2.767971f}, {2.402318f, 2.947488f}, {2.621315f, 3.127540f}, {2.759315f, 3.306994f},
    {2.919486f, 3.485407f}, {3.116810f, 3.664799f}, {3.384219f, 3.845500f}, {3.654829f, 4.024267f},
    {3.905801f, 4.204214f}, {4.128683f, 4.384434f}, {4.349133f, 4.562612f}, {4.543364f, 4.743505f},
    {4.544728f, 4.741508f}, {4.668600f, 4.921982f}, {4.837042f, 5.102610f}, {5.039588f, 5.280833f},
    {5.290755f, 5.460778f}, {5.568966f, 5.641462f}, {5.810764f, 5.819294f}, {6.023868f, 6.000088f},
    {6.209723f, 6.180309f}, {6.283185f, 6.283185f}};

inline const tap::algorithms::transforms::Transform TURRET_MAJOR_IMU_MOUNTING_TRANSFORM(
    0,
    0,
    0,
    M_PI,
    0,
    0);

namespace chassisFrameController
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 58'472.8047f,
    .ki = 100'000.0f,
    .kd = 6'423.45311f,
    .maxICumulative = 3'000.0f,
    .maxOutput = static_cast<uint16_t>(tap::motor::DjiMotor::MAX_OUTPUT_C620),
    .tQDerivativeKalman = 10.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

}  // namespace chassisFrameController

namespace worldFrameCascadeController
{
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.2f,
    .maxOutput = 20.0f,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 2500.0f,
    .ki = 10'000.0f,
    .kd = 10.0f,
    .maxICumulative = 1'500.0f,
    .maxOutput = static_cast<uint16_t>(tap::motor::DjiMotor::MAX_OUTPUT_C620 * 1),
    .tRDerivativeKalman = 60'000.0f,  // Gain needs to be so high for the motors to actually do
                                      // anything that motor encoder resolution becomes a problem
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 60.0f,
    .errDeadzone = 0.0f,
};

}  // namespace worldFrameCascadeController

// Turret Major has a double DJI motor, so we need to have two CAN Buses
static constexpr tap::can::CanBus CAN_BUS_MOTOR = tap::can::CanBus::CAN_BUS2;

static constexpr float MAX_VEL_ERROR_INPUT = 20.0f;
}  // namespace turretMajor

static constexpr float ANGLES_OF_FREEDOM = modm::toRadian(334);
static constexpr float PADDING = modm::toRadian(5);
namespace turretWidow
{
static constexpr uint8_t turretID = 0;

static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR5;

static constexpr float CENTER_OF_FREEDOM = modm::toRadian(0);

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 2693,
    .minAngle = CENTER_OF_FREEDOM - ANGLES_OF_FREEDOM / 2.f + PADDING,
    .maxAngle = CENTER_OF_FREEDOM + ANGLES_OF_FREEDOM / 2.f - PADDING,
    .limitMotorAngles = true,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 7357,
    .minAngle = modm::toRadian(-13),  // actual CAD limit is -14
    .maxAngle = modm::toRadian(40),   // actual CAD limit is -48
    .limitMotorAngles = true,
};

static constexpr float majorToTurretR = 0.145;
static constexpr float DEFAULT_LAUNCH_SPEED = 25.0f;

static constexpr tap::communication::serial::RefSerial::Rx::MechanismID barrelID =
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM;

inline const tap::algorithms::transforms::Transform TURRET_MCB1_BMI088_MOUNTING_TRANSFORM(
    0.0f,
    0.20667f,
    0.0f,
    0.0f,
    0.0f,
    -3.1415926535f);

inline const tap::algorithms::transforms::Transform TURRET_MCB1_ISM330_MOUNTING_TRANSFORM(
    0.00868f,
    0.1734f,
    0.0f,
    0.0f,
    0.0f,
    -3.1415926535f * .5f);

inline constexpr float TORQUE_TO_MOTOR_OUTPUT = 1 / DESIRED_OUT_TO_TORQUE;

inline constexpr aruwsrc::control::turret::algorithms::OptimalSTOSController::STOSConstants
    turretWidowSTOSConstants = {
        .J_TOTAL = 0.0073f,
        .TAU_MAX = 1.3f,
        .B_DAMP = 0.001f,
        .W_D = 80.8f,
        .ZETA = 0.33,
        .SYSTEM_DELAY_SEC = 0.008f,
        .TorqueToMotorOutput = TORQUE_TO_MOTOR_OUTPUT};

inline constexpr aruwsrc::control::turret::algorithms::TurretFeedforwardConstants
    turretWidowFeedforwardConstants = {
        .Ka = 0.0073f * TORQUE_TO_MOTOR_OUTPUT,
        .Kv = 0.0443f * TORQUE_TO_MOTOR_OUTPUT,
        .Ks = 0.001f * TORQUE_TO_MOTOR_OUTPUT};
}  // namespace turretWidow

namespace minorPidConfigs
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG_CHASSIS_FRAME = {
    .kp = 70'000.0f,
    .ki = 5'000.0f,
    .kd = 3'500.0f,
    .maxICumulative = 250.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 80.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG_CHASSIS_FRAME = {
    .kp = 80'000.0f,
    .ki = 100'000.0f,
    .kd = 3'000.0f,
    .maxICumulative = 6'000.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 60.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .antiSaturation = true,
};

// static constexpr tap::algorithms::SmoothPidConfig MINOR_YAW_PID_CONFIG_WORLD_FRAME_VEL = {
//     .kp = 3'750.0f,
//     .ki = 0.0f,
//     .kd = 0.010f,
//     .maxICumulative = 0.0f,
//     .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
//     .tQDerivativeKalman = 1.0f,
//     .tRDerivativeKalman = 0.0f,
//     .tQProportionalKalman = 1.0f,
//     .tRProportionalKalman = 0.5f,
//     .errDeadzone = 0.0f,
// };

static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG_WORLD_FRAME_POS = {
    .kp = 63'000,
    .ki = 0.0f,
    .kd = 3'150.0f,
    .maxICumulative = 1.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG_WORLD_FRAME_VEL = {
    .kp = 5'000.0f,
    .ki = 40'000.0f,
    .kd = 0.005f,
    .maxICumulative = 100.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG_WORLD_FRAME_POS = {
    .kp = 20.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.05f,
    .maxOutput = 30.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};
}  // namespace minorPidConfigs
}  // namespace  aruwsrc::control::turret

#endif  // SENTRY_TURRET_CONSTANTS_HPP_
