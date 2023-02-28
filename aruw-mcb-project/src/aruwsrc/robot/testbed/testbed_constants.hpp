#ifndef TESTBED_CONSTANTS_HPP_
#define TESTBED_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"

#include "aruwsrc/control/turret/turret_motor_config.hpp"

namespace aruwsrc::control::turret
{
static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 0,
    .minAngle = 0,
    .maxAngle = 0,
    .limitMotorAngles = true,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 0,
    .minAngle = 0,
    .maxAngle = 0,
    .limitMotorAngles = true,
};
static constexpr uint8_t NUM_TURRETS = 1;

static constexpr float USER_YAW_INPUT_SCALAR = 0.02f;
static constexpr float USER_PITCH_INPUT_SCALAR = 0.02f;

static constexpr float TURRET_CG_X = 30.17;
static constexpr float TURRET_CG_Z = 34.02;
static constexpr float GRAVITY_COMPENSATION_SCALAR = 0;
}  // namespace aruwsrc::control::turret

const static tap::algorithms::SmoothPidConfig TURRET_PID_CONFIG = {
    .kp = 100,
    .ki = 0,
    .kd = 0,
};

#endif