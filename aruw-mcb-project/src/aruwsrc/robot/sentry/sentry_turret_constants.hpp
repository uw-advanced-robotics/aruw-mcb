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
#include "aruwsrc/control/turret/algorithms/turret_spring_compensation.hpp"
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
        .cgX = 31.81f,
        .cgZ = -37.36f,
        .gravityCompensatorMax = -8777.7f,
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
static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 1060,
    .minAngle = 0,
    .maxAngle = M_TWOPI,
    .limitMotorAngles = false,
};

static constexpr tap::can::CanBus YAW_ANALOG_SENSOR_CAN_BUS = tap::can::CanBus::CAN_BUS2;
static constexpr uint16_t YAW_ANALOG_SENSOR_CAN_ID = 0x1D6;
static constexpr uint8_t YAW_ANALOG_SENSOR_CHANNEL = 1;  // 0 = AI0, 1 = AI1
static constexpr bool YAW_ANALOG_SENSOR_INVERTED = false;
static constexpr uint16_t YAW_ANALOG_RAW_MIN = 0;
static constexpr uint16_t YAW_ANALOG_RAW_MAX = 10000;
static constexpr uint16_t YAW_ANALOG_RAW_ZERO = 4550;
static constexpr float YAW_ANALOG_OUTPUT_RANGE_RADIANS = M_TWOPI;

static constexpr modm::Pair<float, float> LAMPREY_CALIBRATION_MAP[] = {
    {0.0f, 0.000000000f},    {70.0f, 0.053926382f},   {289.0f, 0.230385222f},
    {541.0f, 0.414017592f},  {588.0f, 0.423332489f},  {816.0f, 0.595537759f},
    {1073.0f, 0.772367484f}, {1311.0f, 0.951635755f}, {1557.0f, 1.131002224f},
    {1783.0f, 1.310205198f}, {2114.0f, 1.489850019f}, {2305.0f, 1.672643039f},
    {2515.0f, 1.854085659f}, {2747.0f, 2.033110839f}, {3058.0f, 2.209519599f},
    {3395.0f, 2.391769859f}, {3709.0f, 2.568024849f}, {4014.0f, 2.746368619f},
    {4322.0f, 2.924789409f}, {4680.0f, 3.106335379f}, {4859.0f, 3.285434699f},
    {5079.0f, 3.465885859f}, {5366.0f, 3.648361659f}, {5687.0f, 3.826956249f},
    {6114.0f, 4.006152129f}, {6507.0f, 4.185609319f}, {6886.0f, 4.364767049f},
    {7210.0f, 4.541015839f}, {7636.0f, 4.717918139f}, {7727.0f, 4.901065089f},
    {8012.0f, 5.081168389f}, {8282.0f, 5.262635449f}, {8581.0f, 5.442081669f},
    {8922.0f, 5.621436339f}, {9232.0f, 5.800632689f}, {9546.0f, 5.979011279f},
    {9839.0f, 6.159154629f}, {10000.0f, 6.283185307f}};

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
    .kp = 160000.0f,
    .ki = 0.0f,
    .kd = 8000.0f,
    .maxICumulative = 10000.0f,
    .maxOutput = static_cast<uint16_t>(tap::motor::DjiMotor::MAX_OUTPUT_C620 * 0.2),
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
    .kp = 4000.0f,
    .ki = 0.0f,
    .kd = 500.0f,
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
static constexpr float TURRET_MINOR_TORQUE_RATIO = 0.0f;

static constexpr float FEEDFORWARD_GAIN = 0.0f;
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
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_2;

static const tap::algorithms::transforms::Transform TURRET_MCB1_BMI088_MOUNTING_TRANSFORM(
    0.0f,
    0.20667f,
    0.0f,
    0.0f,
    0.0f,
    0.0f);

static const tap::algorithms::transforms::Transform TURRET_MCB1_ISM330_MOUNTING_TRANSFORM(
    0.00868f,
    0.1734f,
    0.0f,
    0.0f,
    0.0f,
    0.0f);
}  // namespace turretWidow

namespace minorPidConfigs
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG_CHASSIS_FRAME = {
    .kp = 50000.0f,
    .ki = 0.0f,
    .kd = 3000.0f,
    .maxICumulative = 2'000.0f,
    .maxOutput = 15'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 80.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG_CHASSIS_FRAME = {
    .kp = 80000.0f,
    .ki = 0.0f,
    .kd = 3000.0f,
    .maxICumulative = 4'000.0f,
    .maxOutput = 15'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 60.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig MINOR_YAW_PID_CONFIG_WORLD_FRAME_VEL = {
    .kp = 3'000.0f,
    .ki = 0.0f,
    .kd = 10.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020_mA,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG_WORLD_FRAME_POS = {
    .kp = 20.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 1.0f,
    .maxOutput = 30.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG_WORLD_FRAME_VEL = {
    .kp = 3'000.0f,
    .ki = 40.0f,
    .kd = 5.0f,
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
