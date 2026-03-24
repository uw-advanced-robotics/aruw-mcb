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

#include "tap/algorithms/math_user_utils.hpp"

namespace aruwsrc::control::motor
{

DamiaoMotor::DamiaoMotor(
    tap::Drivers* drivers,
    DamiaoMotorId motorId,
    tap::can::CanBus motorCanBus,
    bool isInverted,
    const char* name,
    uint16_t feedbackCanId)
    : CanRxListener(drivers, feedbackCanId, motorCanBus),
      drivers(drivers),
      motorIdentifier(static_cast<uint16_t>(motorId)),
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
    const int32_t signedOutput = motorInverted ? -desiredOutput : desiredOutput;
    desiredOutputMilliNm = tap::algorithms::limitVal<int32_t>(signedOutput, -10000, 10000);

    // Retry startup handshake while offline. Control mode is configured on the motor itself;
    // the runtime 0x7FF register write was causing the bus to stall before enable/MIT frames
    // were ever transmitted.
    if (!isMotorOnline() && (startupRetryTimeout.isStopped() || startupRetryTimeout.isExpired()))
    {
        clearError();
        enable();
        startupRetryTimeout.restart(STARTUP_RETRY_PERIOD_MS);
    }

    // Always send MIT command every frame so motor gets constant position/torque updates.
    const float torqueNm = static_cast<float>(desiredOutputMilliNm) / 1000.0f;
    sendMitCommand(0.0f, 0.0f, 0.0f, 0.0f, torqueNm);
}

bool DamiaoMotor::isMotorOnline() const
{
    return !motorDisconnectTimeout.isExpired() && !motorDisconnectTimeout.isStopped();
}

int16_t DamiaoMotor::getOutputDesired() const
{
    return static_cast<int16_t>(desiredOutputMilliNm);
}

int8_t DamiaoMotor::getTemperature() const { return temperatureMos; }

int16_t DamiaoMotor::getTorque() const { return measuredTorqueMilliNm; }

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

    const uint16_t posInt = static_cast<uint16_t>((message.data[1] << 8) | message.data[2]);
    const uint16_t velInt = static_cast<uint16_t>((message.data[3] << 4) | (message.data[4] >> 4));
    const uint16_t torqueInt =
        static_cast<uint16_t>(((message.data[4] & 0x0F) << 8) | message.data[5]);

    const float posRad = uintToFloat(posInt, DM4310_P_MIN, DM4310_P_MAX, 16);
    const float velRadPerSec = uintToFloat(velInt, DM4310_V_MIN, DM4310_V_MAX, 12);
    const float torqueNm = uintToFloat(torqueInt, DM4310_T_MIN, DM4310_T_MAX, 12);

    encoder.updateFromFeedback(posRad, velRadPerSec);
    measuredTorqueMilliNm = static_cast<int16_t>(torqueNm * 1000.0f);
    temperatureMos = static_cast<int8_t>(message.data[6]);

    motorDisconnectTimeout.restart(MOTOR_DISCONNECT_TIME_MS);
}

bool DamiaoMotor::enable()
{
    static constexpr uint8_t ENABLE_DATA[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    return sendRawFrame(ENABLE_DATA, 8, motorIdentifier);
}

bool DamiaoMotor::disable()
{
    static constexpr uint8_t DISABLE_DATA[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    return sendRawFrame(DISABLE_DATA, 8, motorIdentifier);
}

bool DamiaoMotor::clearError()
{
    static constexpr uint8_t CLEAR_ERROR_DATA[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    return sendRawFrame(CLEAR_ERROR_DATA, 8, motorIdentifier);
}

uint16_t DamiaoMotor::floatToUint(float x, float xMin, float xMax, uint8_t bits)
{
    const float span = xMax - xMin;
    const float xClipped = tap::algorithms::limitVal<float>(x, xMin, xMax);
    const float scaled = (xClipped - xMin) * ((1u << bits) - 1u) / span;
    return static_cast<uint16_t>(scaled);
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
    const uint16_t posU = floatToUint(targetPosition, DM4310_P_MIN, DM4310_P_MAX, 16);
    const uint16_t velU = floatToUint(targetVelocity, DM4310_V_MIN, DM4310_V_MAX, 12);
    const uint16_t kpU = floatToUint(kp, 0.0f, 500.0f, 12);
    const uint16_t kdU = floatToUint(kd, 0.0f, 5.0f, 12);
    const uint16_t torqueU = floatToUint(torqueNm, DM4310_T_MIN, DM4310_T_MAX, 12);

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

    return sendRawFrame(data, 8, motorIdentifier);
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
