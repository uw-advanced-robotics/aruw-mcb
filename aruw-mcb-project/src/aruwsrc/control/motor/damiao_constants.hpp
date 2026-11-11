#ifndef DAMIAO_CONSTANTS_HPP_
#define DAMIAO_CONSTANTS_HPP_

#include <cassert>
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

enum DamiaoMotorType : uint16_t
{
    DAMIAO_4310,
    DAMIAO_2325,
    DAMIAO_4340,
};

struct DamiaoMotorLimits
{
    float pmin;
    float pmax;
    float vmin;
    float vmax;
    float tmin;
    float tmax;
    int32_t maxOutputMilliNm;
};

// Mapping ranges must match the motor's PMAX, VMAX, and TMAX registers.
static constexpr DamiaoMotorLimits DM_J4310_LIMITS =
    {-12.5f, 12.5f, -30.0f, 30.0f, -10.0f, 10.0f, 10000};
static constexpr DamiaoMotorLimits DM_J4340_LIMITS =
    {-12.5f, 12.5f, -10.0f, 10.0f, -28.0f, 28.0f, 28000};
// shortcut: 2325 values are J4310 placeholders; replace with verified motor settings before use.
static constexpr DamiaoMotorLimits DM_J2325_LIMITS =
    {-12.5f, 12.5f, -30.0f, 30.0f, -10.0f, 10.0f, 10000};

constexpr DamiaoMotorLimits getDamiaoMotorLimits(DamiaoMotorType type)
{
    switch (type)
    {
        case DAMIAO_4310:
            return DM_J4310_LIMITS;
        case DAMIAO_2325:
            return DM_J2325_LIMITS;
        case DAMIAO_4340:
            return DM_J4340_LIMITS;
    }
    assert(false && "Invalid Damiao motor type");
    return DM_J4310_LIMITS;
}

static constexpr float DAMIAO_KP_MIN = 0.0f;
static constexpr float DAMIAO_KP_MAX = 500.0f;
static constexpr float DAMIAO_KD_MIN = 0.0f;
static constexpr float DAMIAO_KD_MAX = 5.0f;
}  // namespace aruwsrc::control::motor

#endif  // DAMIAO_CONSTANTS_HPP_
