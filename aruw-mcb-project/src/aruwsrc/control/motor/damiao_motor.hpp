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

#include "damiao_constants.hpp"

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

/**
 * DaMiao CAN motor driver supporting MIT torque, position-speed, and velocity control.
 *
 * The frame layout follows the DM-J4310 documentation:
 * - MIT command (8 bytes): pos[16], vel[12], kp[12], kd[12], torque[12]
 * - Position-speed command (8 bytes): little-endian position and velocity floats
 * - Velocity command (4 bytes): little-endian velocity float
 * - Feedback frame (8 bytes): id/status nibble + pos/vel/torque + temperatures
 */
class DamiaoMotor : public tap::can::CanRxListener, public tap::motor::MotorInterface
{
public:
    // Retain the existing public names for the shared DM-J4310 limits.

    /**
     * Constructor for a Damiao motor.
     *
     * @param drivers The drivers singleton.
     * @param motorId The motor ID.
     * @param motorCanBus The motor CAN bus.
     * @param isInverted Whether the motor is inverted.
     * @param name The name of the motor.
     * @param feedbackCanId The CAN ID of the feedback frame.
     * @param controlMode The control mode of the motor, defaults to MIT mode (torque control).
     * Must match the mode configured on the motor using the debugging assistant.
     */
    DamiaoMotor(
        tap::Drivers* drivers,
        DamiaoMotorId motorId,
        tap::can::CanBus motorCanBus,
        bool isInverted,
        const char* name,
        uint16_t feedbackCanId = 0x000,
        DamiaoControlMode controlMode = DamiaoControlMode::DAMIAO_CONTROL_MODE_MIT);

    ~DamiaoMotor();

    void initialize() override;

    tap::encoder::EncoderInterface* getEncoder() const override
    {
        return const_cast<DamiaoMotorEncoder*>(&encoder);
    }

    // Torque in milli-Nm; only used in MIT mode.
    void setDesiredOutput(int32_t desiredOutput) override;

    bool isMotorOnline() const override;

    int16_t getOutputDesired() const override;

    int8_t getTemperature() const override;

    int16_t getTorque() const override;

    float getPosition() const;

    // Feedback status nibble: 0 disabled, 1 enabled, 8..E faults; FF before feedback.
    uint8_t getStatus() const { return motorStatus; }

    void processMessage(const modm::can::Message& message) override;

    // Position in radians and maximum speed in rad/s. Call each control cycle.
    // Returns false if the selected mode does not match or CAN transmission fails.
    bool sendPositionCommand(float targetPosition, float velocityMax);

    // Velocity in rad/s. Call each control cycle; honors motor inversion.
    // Returns false if the selected mode does not match or CAN transmission fails.
    bool sendVelocityCommand(float targetVelocity);

    // Stop in the configured mode: zero torque, zero speed, or current position at zero speed.
    void stop();

    bool enable();

    bool disable();

    bool clearError();

    bool sendMitCommand(
        float targetPosition,
        float targetVelocity,
        float kp,
        float kd,
        float torqueNm);

private:
    static constexpr uint32_t MOTOR_DISCONNECT_TIME_MS = 100;
    static constexpr uint32_t STARTUP_RETRY_PERIOD_MS = 100;

    void retryStartup();

    // 12/16 bits uses MIT scaling; 32 bits preserves raw float encoding.
    template <typename Uint, uint8_t Bits>
    static Uint floatToUint(float x, float xMin, float xMax);

    static float uintToFloat(uint16_t xInt, float xMin, float xMax, uint8_t bits);

    bool sendRawFrame(const uint8_t* data, uint8_t dataLen, uint16_t frameId) const;

    tap::Drivers* drivers;
    uint16_t motorIdentifier;
    DamiaoControlMode controlMode;
    tap::can::CanBus motorCanBus;
    bool motorInverted;

    DamiaoMotorEncoder encoder;
    tap::arch::MilliTimeout motorDisconnectTimeout;
    tap::arch::MilliTimeout startupRetryTimeout;

    int32_t desiredOutputMilliNm = 0;
    uint8_t motorStatus = 0xFF;
    // Preserve the motor's raw coordinate for position-mode stop commands.
    float measuredPositionRad = 0.0f;
    int16_t measuredTorqueMilliNm = 0;
    int8_t temperatureMos = 0;

    // Inspect these in the debugger to distinguish CAN bus failures from rejected feedback.
    uint32_t DEBUG_feedbackFramesReceived = 0;
    uint32_t DEBUG_feedbackFramesAccepted = 0;
    uint32_t DEBUG_feedbackBadLength = 0;
    uint32_t DEBUG_feedbackWrongMotorId = 0;
    uint8_t DEBUG_feedbackFirstByte = 0;
    mutable uint32_t DEBUG_canErrorStatus = 0;
    mutable uint8_t DEBUG_canLastError = 0;
    mutable uint8_t DEBUG_canTxErrorCount = 0;
    mutable uint8_t DEBUG_canRxErrorCount = 0;
    mutable bool DEBUG_canBusOff = false;
    mutable uint32_t DEBUG_canSendFailures = 0;
};

}  // namespace aruwsrc::control::motor

#endif  // DAMIAO_MOTOR_HPP_
