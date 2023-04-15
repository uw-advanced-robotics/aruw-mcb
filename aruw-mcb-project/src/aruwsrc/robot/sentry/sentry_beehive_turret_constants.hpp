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


// new 

// TODO: change this name
static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = M_PI_2,
    .startEncoderValue = 6162,
    .minAngle = 0,
    .maxAngle = M_PI,
    .limitMotorAngles = false,
};

namespace turretMajor
{

// Turret Major has a double DJI motor, so we need to have two CAN Buses
static constexpr tap::can::CanBus CAN_BUS_MOTOR_1 = tap::can::CanBus::CAN_BUS1;
static constexpr tap::can::CanBus CAN_BUS_MOTOR_2 = tap::can::CanBus::CAN_BUS2;
// static constexptr tap::can::CanBus 


}

namespace turretMinor0
{
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
    .startEncoderValue = 3335,
    .minAngle = modm::toRadian(-15),
    .maxAngle = modm::toRadian(45),
    .limitMotorAngles = true,
};

}

namespace turretMinor1
{
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
    .startEncoderValue = 2035,
    .minAngle = modm::toRadian(-15),
    .maxAngle = modm::toRadian(45),
    .limitMotorAngles = true,
};

}




// old 



static constexpr uint8_t NUM_TURRETS = 2;

static constexpr float USER_YAW_INPUT_SCALAR = 0.02f;
static constexpr float USER_PITCH_INPUT_SCALAR = 0.02f;


static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR5;

static constexpr float TURRET_CG_X = -48.14f;
static constexpr float TURRET_CG_Z = 9.45f;
static constexpr float GRAVITY_COMPENSATION_SCALAR = 8'000.0f;

// The distance from turret 0 to turret 1 in meters
static modm::Vector3f OFFSET_TURRET_0_TO_TURRET_1 = modm::Vector3f(-0.17511f, -.27905f, 0.0f);
static constexpr float PITCH_YAW_OFFSET = 0.045f;

namespace chassis_rel
{
namespace turretMajor
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 70'000.0f,
    .ki = 0.0f,
    .kd = 3'000.0f,
    .maxICumulative = 0.0f,
    // .maxOutput = 30'000.0f,
    .maxOutput = 10000.f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};
}  // namespace turretMinor0

namespace turretMinor0
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 70'000.0f,
    .ki = 0.0f,
    .kd = 3'000.0f,
    .maxICumulative = 0.0f,
    // .maxOutput = 30'000.0f,
.maxOutput = 5000.f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 100'000.0f,
    .ki = 50.0f,
    .kd = 2'500.0f,
    .maxICumulative = 3'000.0f,
    .maxOutput = 5000.f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 20.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};
}  // namespace turretMinor0

namespace turretMinor1
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 70'000.0f,
    .ki = 0.0f,
    .kd = 3'000.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 5000.0f,
    // .maxOutput = 30'000.0f
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 40.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 100'000.0f,
    .ki = 50.0f,
    .kd = 2'500.0f,
    .maxICumulative = 3'000.0f,
    .maxOutput = 5000.f,
    // .maxOutput = 30'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 20.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};
}  // namespace turretMinor1
}  // namespace chassis_rel

namespace world_rel_turret_imu
{
namespace turretMinor0
{
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 18.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 5000.0f,
    // .maxOutput = 10'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 30'000.0f,
    .ki = 286.5f,
    .kd = 0.0f,
    .maxICumulative = 2'000.0f,
    .maxOutput =5000.0f,
    // .maxOutput = 30'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
}  // namespace turretMinor0

namespace turretMinor1
{
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 18.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 5000.0f,
    // .maxOutput = 10'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 30'000.0f,
    .ki = 286.5f,
    .kd = 0.0f,
    .maxICumulative = 2'000.0f,
    // .maxOutput = 30'000.0f,
    .maxOutput = 5000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
}  // namespace turretMinor1
}  // namespace world_rel_turret_imu
}  // namespace  aruwsrc::control::turret

#endif  // SENTRY_BEEHIVE_TURRET_CONSTANTS_HPP_
