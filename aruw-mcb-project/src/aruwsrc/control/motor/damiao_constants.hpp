#ifndef DAMIAO_CONSTANTS_HPP_
#define DAMIAO_CONSTANTS_HPP_

#include <cstdint>

namespace aruwsrc::control::motor
{
// CAN command ID offset per control mode (MIT = 0, position-speed = 1, velocity = 2).
static constexpr uint16_t DAMIAO_MODE_ID_OFFSET = 0x100;

// max output for DM-J4310
static constexpr int32_t MAX_OUTPUT_DM_J4310_MILLI_NM = 10000;

// mit mode limits for DM-J4310
static constexpr float DM_J4310_P_MIT_MIN = -12.5f;
static constexpr float DM_J4310_P_MIT_MAX = 12.5f;
static constexpr float DM_J4310_V_MIT_MIN = -30.0f;
static constexpr float DM_J4310_V_MIT_MAX = 30.0f;
static constexpr float DM_J4310_T_MIT_MIN = -10.0f;
static constexpr float DM_J4310_T_MIT_MAX = 10.0f;

static constexpr float DM_J4310_KP_MIN = 0.0f;
static constexpr float DM_J4310_KP_MAX = 500.0f;
static constexpr float DM_J4310_KD_MIN = 0.0f;
static constexpr float DM_J4310_KD_MAX = 5.0f;

// Application limits for position-speed mode; the protocol uses raw floats.
// minimum position in radians
static constexpr float DM_J4310_P_POS_MIN = -12.5f;
static constexpr float DM_J4310_P_POS_MAX = 12.5f;
// maximum velocity in radians per second
static constexpr float DM_J4310_V_POS_MIN = -30.0f;
static constexpr float DM_J4310_V_POS_MAX = 30.0f;

// Application velocity limits in rad/s.
static constexpr float DM_J4310_VEL_MIN = -30.0f;
static constexpr float DM_J4310_VEL_MAX = 30.0f;

}  // namespace aruwsrc::control::motor

#endif  // DAMIAO_CONSTANTS_HPP_
