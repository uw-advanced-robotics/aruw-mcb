/*
 * Copyright (c) 2020-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "damiao_motor.hpp"

#include <sys/types.h>

#include <cstdint>
#include <cstring>
#include <type_traits>

#include "tap/algorithms/math_user_utils.hpp"

#include "arm_math.h"
#include "damiao_constants.hpp"

namespace aruwsrc::control::motor
{
DamiaoMotor::DamiaoMotor(
    tap::Drivers* drivers,
    DamiaoMotorId motorId,
    tap::can::CanBus motorCanBus,
    bool isInverted,
    const char* name,
    uint16_t feedbackCanId,
    DamiaoControlMode controlMode)
    : CanRxListener(drivers, feedbackCanId, motorCanBus),
      drivers(drivers),
      motorIdentifier(static_cast<uint16_t>(motorId)),
      controlMode(controlMode),
      motorCanBus(motorCanBus),
      motorInverted(isInverted),
      encoder(isInverted)
{
    (void)name;
    motorDisconnectTimeout.stop();
    startupRetryTimeout.stop();
}

DamiaoMotor::~DamiaoMotor() {}

void DamiaoMotor::initialize()
{
    attachSelfToRxHandler();
    // Defer startup handshake to the control loop so CAN is definitely up.
    startupRetryTimeout.stop();
}

void DamiaoMotor::setDesiredOutput(int32_t desiredOutput)
{
    if (controlMode != DAMIAO_CONTROL_MODE_MIT)
    {
        return;
    }

    const int32_t signedOutput = motorInverted ? -desiredOutput : desiredOutput;
    desiredOutputMilliNm = tap::algorithms::limitVal<int32_t>(
        signedOutput,
        -MAX_OUTPUT_DM_J4310_MILLI_NM,
        MAX_OUTPUT_DM_J4310_MILLI_NM);

    retryStartup();

    // Always send MIT command every frame so motor gets constant position/torque updates.
    const float torqueNm = static_cast<float>(desiredOutputMilliNm) / 1000.0f;
    sendMitCommand(0.0f, 0.0f, 0.0f, 0.0f, torqueNm);
}

void DamiaoMotor::retryStartup()
{
    // Retry startup handshake while offline. Control mode is configured on the motor itself;
    // the runtime 0x7FF register write was causing the bus to stall before enable/control frames
    // were ever transmitted.
    if ((!isMotorOnline() || motorStatus == 0) &&
        (startupRetryTimeout.isStopped() || startupRetryTimeout.isExpired()))
    {
        clearError();
        enable();
        startupRetryTimeout.restart(STARTUP_RETRY_PERIOD_MS);
    }
}

bool DamiaoMotor::isMotorOnline() const
{
    return !motorDisconnectTimeout.isExpired() && !motorDisconnectTimeout.isStopped();
}

int16_t DamiaoMotor::getOutputDesired() const { return static_cast<int16_t>(desiredOutputMilliNm); }

int8_t DamiaoMotor::getTemperature() const { return temperatureMos; }

int16_t DamiaoMotor::getTorque() const { return measuredTorqueMilliNm; }

float DamiaoMotor::getPosition() const { return encoder.getPosition().getUnwrappedValue(); }

void DamiaoMotor::processMessage(const modm::can::Message& message)
{
    if (message.getLength() != 8)
    {
        return;
    }

    const uint8_t logicalId = message.data[0] & 0x0F;
    if (logicalId != (motorIdentifier & 0x0F))
    {
        return;
    }

    motorStatus = message.data[0] >> 4;
    const uint16_t posInt = static_cast<uint16_t>((message.data[1] << 8) | message.data[2]);
    const uint16_t velInt = static_cast<uint16_t>((message.data[3] << 4) | (message.data[4] >> 4));
    const uint16_t torqueInt =
        static_cast<uint16_t>(((message.data[4] & 0x0F) << 8) | message.data[5]);

    const float posRad = uintToFloat(posInt, DM_J4310_P_MIN, DM_J4310_P_MAX, 16);
    const float velRadPerSec = uintToFloat(velInt, DM_J4310_V_MIN, DM_J4310_V_MAX, 12);
    const float torqueNm = uintToFloat(torqueInt, DM_J4310_T_MIN, DM_J4310_T_MAX, 12);

    measuredPositionRad = posRad;
    encoder.updateFromFeedback(posRad, velRadPerSec);
    measuredTorqueMilliNm = static_cast<int16_t>(torqueNm * 1000.0f);
    temperatureMos = static_cast<int8_t>(message.data[6]);

    motorDisconnectTimeout.restart(MOTOR_DISCONNECT_TIME_MS);
}

bool DamiaoMotor::enable()
{
    static constexpr uint8_t ENABLE_DATA[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    return sendRawFrame(ENABLE_DATA, 8, motorIdentifier + static_cast<uint16_t>(controlMode));
}

bool DamiaoMotor::disable()
{
    static constexpr uint8_t DISABLE_DATA[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    return sendRawFrame(DISABLE_DATA, 8, motorIdentifier + static_cast<uint16_t>(controlMode));
}

bool DamiaoMotor::clearError()
{
    static constexpr uint8_t CLEAR_ERROR_DATA[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    return sendRawFrame(CLEAR_ERROR_DATA, 8, motorIdentifier + static_cast<uint16_t>(controlMode));
}

template <typename Uint, uint8_t Bits>
Uint DamiaoMotor::floatToUint(float x, float xMin, float xMax)
{
    static_assert(std::is_same_v<Uint, uint16_t> || std::is_same_v<Uint, uint32_t>);
    static_assert(Bits == 12 || Bits == 16 || Bits == 32);
    static_assert(Bits <= sizeof(Uint) * 8);

    const float xClipped = tap::algorithms::limitVal<float>(x, xMin, xMax);
    if constexpr (Bits == 32)
    {
        // Position-speed and velocity commands carry raw IEEE-754 floats.
        static_assert(sizeof(float) == sizeof(Uint));
        Uint raw;
        std::memcpy(&raw, &xClipped, sizeof(raw));
        return raw;
    }
    else
    {
        // MIT commands carry scaled 12-bit or 16-bit unsigned integers.
        const float span = xMax - xMin;
        const float scaled = (xClipped - xMin) * ((1u << Bits) - 1u) / span;
        return static_cast<Uint>(scaled);
    }
}

float DamiaoMotor::uintToFloat(uint16_t xInt, float xMin, float xMax, uint8_t bits)
{
    const float span = xMax - xMin;
    return static_cast<float>(xInt) * span / static_cast<float>((1u << bits) - 1u) + xMin;
}

bool DamiaoMotor::sendMitCommand(
    float targetPosition,
    float targetVelocity,
    float kp,
    float kd,
    float torqueNm)
{
    if (controlMode != DAMIAO_CONTROL_MODE_MIT)
    {
        return false;
    }
    const uint16_t posU = floatToUint<uint16_t, 16>(targetPosition, DM_J4310_P_MIN, DM_J4310_P_MAX);
    const uint16_t velU = floatToUint<uint16_t, 12>(targetVelocity, DM_J4310_V_MIN, DM_J4310_V_MAX);
    const uint16_t kpU = floatToUint<uint16_t, 12>(kp, DM_J4310_KP_MIN, DM_J4310_KP_MAX);
    const uint16_t kdU = floatToUint<uint16_t, 12>(kd, DM_J4310_KD_MIN, DM_J4310_KD_MAX);
    const uint16_t torqueU = floatToUint<uint16_t, 12>(torqueNm, DM_J4310_T_MIN, DM_J4310_T_MAX);

    uint8_t data[8] = {
        static_cast<uint8_t>((posU >> 8) & 0xFF),
        static_cast<uint8_t>(posU & 0xFF),
        static_cast<uint8_t>((velU >> 4) & 0xFF),
        static_cast<uint8_t>(((velU & 0x0F) << 4) | ((kpU >> 8) & 0x0F)),
        static_cast<uint8_t>(kpU & 0xFF),
        static_cast<uint8_t>((kdU >> 4) & 0xFF),
        static_cast<uint8_t>(((kdU & 0x0F) << 4) | ((torqueU >> 8) & 0x0F)),
        static_cast<uint8_t>(torqueU & 0xFF),
    };

    return sendRawFrame(data, 8, motorIdentifier + static_cast<uint16_t>(controlMode));
}

bool DamiaoMotor::sendPositionCommand(float32_t targetPosition, float32_t targetVelocity)
{
    if (controlMode != DAMIAO_CONTROL_MODE_POSITION_SPEED)
    {
        return false;
    }

    retryStartup();
    const uint32_t posU = floatToUint<uint32_t, 32>(targetPosition, DM_J4310_P_MIN, DM_J4310_P_MAX);
    const uint32_t velU = floatToUint<uint32_t, 32>(targetVelocity, DM_J4310_V_MIN, DM_J4310_V_MAX);

    // Position-speed commands use little-endian IEEE-754 floats (manual, page 10).
    uint8_t data[8] = {
        static_cast<uint8_t>(posU & 0xFF),
        static_cast<uint8_t>((posU >> 8) & 0xFF),
        static_cast<uint8_t>((posU >> 16) & 0xFF),
        static_cast<uint8_t>((posU >> 24) & 0xFF),
        static_cast<uint8_t>(velU & 0xFF),
        static_cast<uint8_t>((velU >> 8) & 0xFF),
        static_cast<uint8_t>((velU >> 16) & 0xFF),
        static_cast<uint8_t>((velU >> 24) & 0xFF),
    };

    return sendRawFrame(data, 8, motorIdentifier + static_cast<uint16_t>(controlMode));
}

bool DamiaoMotor::sendVelocityCommand(float targetVelocity)
{
    if (controlMode != DAMIAO_CONTROL_MODE_VELOCITY)
    {
        return false;
    }

    retryStartup();
    const uint32_t velU = floatToUint<uint32_t, 32>(
        motorInverted ? -targetVelocity : targetVelocity,
        DM_J4310_V_MIN,
        DM_J4310_V_MAX);

    // Velocity commands use a little-endian IEEE-754 float (manual, page 11).
    uint8_t data[4] = {
        static_cast<uint8_t>(velU & 0xFF),
        static_cast<uint8_t>((velU >> 8) & 0xFF),
        static_cast<uint8_t>((velU >> 16) & 0xFF),
        static_cast<uint8_t>((velU >> 24) & 0xFF),
    };

    return sendRawFrame(data, 4, motorIdentifier + static_cast<uint16_t>(controlMode));
}

void DamiaoMotor::stop()
{
    switch (controlMode)
    {
        case DAMIAO_CONTROL_MODE_MIT:
            setDesiredOutput(0);
            break;
        case DAMIAO_CONTROL_MODE_VELOCITY:
            sendVelocityCommand(0.0f);
            break;
        case DAMIAO_CONTROL_MODE_POSITION_SPEED:
            if (isMotorOnline())
            {
                sendPositionCommand(measuredPositionRad, 0.0f);
            }
            else
            {
                // Do not command an unknown position when feedback is unavailable.
                disable();
            }
            break;
    }
}

bool DamiaoMotor::sendRawFrame(const uint8_t* data, uint8_t dataLen, uint16_t frameId) const
{
    modm::can::Message message(frameId, dataLen);
    message.setExtended(false);
    message.setRemoteTransmitRequest(false);
    for (uint8_t i = 0; i < dataLen; i++)
    {
        message.data[i] = data[i];
    }

    return drivers->can.sendMessage(motorCanBus, message);
}

}  // namespace aruwsrc::control::motor
