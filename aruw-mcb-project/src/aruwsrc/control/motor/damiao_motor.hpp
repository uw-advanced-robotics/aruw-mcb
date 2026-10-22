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

#ifndef DAMIAO_MOTOR_HPP_
#define DAMIAO_MOTOR_HPP_

#include <cstdint>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/wrapped_float.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/communication/sensors/encoder/encoder_interface.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/motor_interface.hpp"

namespace aruwsrc::control::motor
{
class DamiaoMotorEncoder : public tap::encoder::EncoderInterface
{
public:
    explicit DamiaoMotorEncoder(bool inverted) : position(0.0f, -M_PI, M_PI), inverted(inverted)
    {
        disconnectTimeout.stop();
    }

    void initialize() override {}

    bool isOnline() const override
    {
        return !disconnectTimeout.isExpired() && !disconnectTimeout.isStopped();
    }

    void resetEncoderValue(float pos = 0) override { position.setUnwrappedValue(pos); }

    tap::algorithms::WrappedFloat getPosition() const override { return position; }

    float getVelocity() const override { return velocity; }

    void alignWith(tap::encoder::EncoderInterface* other) override
    {
        if (other != nullptr)
        {
            position.setUnwrappedValue(other->getPosition().getUnwrappedValue());
        }
    }

    void updateFromFeedback(float posRad, float velRadPerSec)
    {
        const float signedPos = inverted ? -posRad : posRad;
        const float signedVel = inverted ? -velRadPerSec : velRadPerSec;
        position.setUnwrappedValue(
            position.getUnwrappedValue() + position.minDifference(signedPos));
        velocity = signedVel;
        disconnectTimeout.restart(MOTOR_DISCONNECT_TIME_MS);
    }

private:
    static constexpr uint32_t MOTOR_DISCONNECT_TIME_MS = 100;
    tap::algorithms::WrappedFloat position;
    float velocity = 0.0f;
    bool inverted;
    tap::arch::MilliTimeout disconnectTimeout;
};

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

/**
 * DaMiao CAN motor driver implementing MIT-mode torque control.
 *
 * The frame layout follows the DM-J4310 documentation:
 * - Command frame (8 bytes): pos[16], vel[12], kp[12], kd[12], torque[12]
 * - Feedback frame (8 bytes): id/status nibble + pos/vel/torque + temperatures
 */
class DamiaoMotor : public tap::can::CanRxListener, public tap::motor::MotorInterface
{
public:
    // DM-J4310 MIT mode limits.
    static constexpr float DM_J4310_P_MIN = -12.5f;
    static constexpr float DM_J4310_P_MAX = 12.5f;
    static constexpr float DM_J4310_V_MIN = -30.0f;
    static constexpr float DM_J4310_V_MAX = 30.0f;
    static constexpr float DM_J4310_T_MIN = -10.0f;
    static constexpr float DM_J4310_T_MAX = 10.0f;
    static constexpr int32_t MAX_OUTPUT_DM_J4310_MILLI_NM = 10000;

    DamiaoMotor(
        tap::Drivers* drivers,
        DamiaoMotorId motorId,
        tap::can::CanBus motorCanBus,
        bool isInverted,
        const char* name,
        uint16_t feedbackCanId = 0x000);

    ~DamiaoMotor();

    void initialize() override;

    tap::encoder::EncoderInterface* getEncoder() const override
    {
        return const_cast<DamiaoMotorEncoder*>(&encoder);
    }

    void setDesiredOutput(int32_t desiredOutput) override;

    bool isMotorOnline() const override;

    int16_t getOutputDesired() const override;

    int8_t getTemperature() const override;

    int16_t getTorque() const override;

    void processMessage(const modm::can::Message& message) override;

    bool enable();

    bool disable();

    bool clearError();

private:
    static constexpr uint32_t MOTOR_DISCONNECT_TIME_MS = 100;
    static constexpr uint32_t STARTUP_RETRY_PERIOD_MS = 100;

    static uint16_t floatToUint(float x, float xMin, float xMax, uint8_t bits);

    static float uintToFloat(uint16_t xInt, float xMin, float xMax, uint8_t bits);

    bool sendMitCommand(
        float targetPosition,
        float targetVelocity,
        float kp,
        float kd,
        float torqueNm);

    bool sendRawFrame(const uint8_t* data, uint8_t dataLen, uint16_t frameId) const;

    tap::Drivers* drivers;
    uint16_t motorIdentifier;
    tap::can::CanBus motorCanBus;
    bool motorInverted;

    DamiaoMotorEncoder encoder;
    tap::arch::MilliTimeout motorDisconnectTimeout;
    tap::arch::MilliTimeout startupRetryTimeout;

    int32_t desiredOutputMilliNm = 0;
    int16_t measuredTorqueMilliNm = 0;
    int8_t temperatureMos = 0;
};

}  // namespace aruwsrc::control::motor

#endif  // DAMIAO_MOTOR_HPP_
