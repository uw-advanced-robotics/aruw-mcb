#ifndef DAMIAO_CONSTANTS_HPP_
#define DAMIAO_CONSTANTS_HPP_

#include <cstdint>

namespace aruwsrc::control::motor
{

enum DamiaoMotorId : uint16_t
{
    DAMIAO_MOTOR1 = 0x001,
    DAMIAO_MOTOR2 = 0x002,
    DAMIAO_MOTOR3 = 0x003,
    DAMIAO_MOTOR4 = 0x004,
    DAMIAO_MOTOR5 = 0x005,
    DAMIAO_MOTOR6 = 0x006,
    DAMIAO_MOTOR7 = 0x007,
    DAMIAO_MOTOR8 = 0x008,
};

// numerical values are the offset from base id
enum DamiaoControlMode : uint16_t
{
    DAMIAO_CONTROL_MODE_MIT = 0x000,
    DAMIAO_CONTROL_MODE_POSITION_SPEED = 0x100,
    DAMIAO_CONTROL_MODE_VELOCITY = 0x200,
};

// max output for DM-J4310
static constexpr int32_t MAX_OUTPUT_DM_J4310_MILLI_NM = 10000;

// limits for position, velocity, and torque
static constexpr float DM_J4310_P_MIN = -12.5f;
static constexpr float DM_J4310_P_MAX = 12.5f;
static constexpr float DM_J4310_V_MIN = -30.0f;
static constexpr float DM_J4310_V_MAX = 30.0f;
static constexpr float DM_J4310_T_MIN = -10.0f;
static constexpr float DM_J4310_T_MAX = 10.0f;

static constexpr float DM_J4310_KP_MIN = 0.0f;
static constexpr float DM_J4310_KP_MAX = 500.0f;
static constexpr float DM_J4310_KD_MIN = 0.0f;
static constexpr float DM_J4310_KD_MAX = 5.0f;

struct DamiaoMotorLimits
{
    float pmin = 0.0f;
    float pmax = 0.0f;
    float vmin = 0.0f;
    float vmax = 0.0f;
    float tmin = 0.0f;
    float tmax = 0.0f;
    float kpmin = 0.0f;
    float kpmax = 0.0f;
    float kdmin = 0.0f;
    float kdmax = 0.0f;
};
}  // namespace aruwsrc::control::motor

#endif  // DAMIAO_CONSTANTS_HPP_
