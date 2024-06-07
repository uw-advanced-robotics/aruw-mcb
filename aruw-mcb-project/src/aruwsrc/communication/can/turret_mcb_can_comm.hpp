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

#ifndef TURRET_MCB_CAN_COMM_HPP_
#define TURRET_MCB_CAN_COMM_HPP_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"
#include "tap/communication/sensors/limit_switch/limit_switch_interface.hpp"

#include "modm/architecture/interface/register.hpp"
#include "modm/math/geometry/angle.hpp"

namespace modm::can
{
class Message;
}

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::can
{
/**
 * A CAN message handler that handles sending and receiving data from the turret mounted
 * microcontroller. Reads IMU data and sends instructions to the turret microcontroller. Follows the
 * protocol described in the wiki here:
 * https://gitlab.com/aruw/controls/aruw-mcb/-/wikis/Turret-MCB-Comm-Protocol.
 *
 * @note Since we use radians in this codebase, angle values that are sent from the turret MCB in
 * degrees are converted to radians by this object.
 */
class TurretMCBCanComm : public tap::communication::sensors::limit_switch::LimitSwitchInterface
{
public:
    using ImuDataReceivedCallbackFunc = void (*)();

    enum class TxCommandMsgBitmask : uint8_t
    {
        OPEN_HOPPER = modm::Bit0,
        RECALIBRATE_IMU = modm::Bit1,
        TURN_LASER_ON = modm::Bit2,
    };
    MODM_FLAGS8(TxCommandMsgBitmask);

    enum CanIDs
    {
        SYNC_RX_CAN_ID = 0x1f8,
        SYNC_TX_CAN_ID = 0x1f9,
        TURRET_STATUS_RX_CAN_ID = 0x1fa,
        X_AXIS_RX_CAN_ID = 0x1fb,
        Y_AXIS_RX_CAN_ID = 0x1fc,
        Z_AXIS_RX_CAN_ID = 0x1fd,
        TURRET_MCB_TX_CAN_ID = 0x1fe,
    };

    TurretMCBCanComm(tap::Drivers* drivers, tap::can::CanBus canBus);
    DISALLOW_COPY_AND_ASSIGN(TurretMCBCanComm);

    mockable void init();

    mockable inline void attachImuDataReceivedCallback(ImuDataReceivedCallbackFunc func)
    {
        imuDataReceivedCallbackFunc = func;
    }

    /**
     * @return turret yaw angle in radians, normalized between [-pi, pi]
     */
    mockable inline float getRoll() const { return lastCompleteImuData.roll; }

    /**
     * @return turret yaw angular velocity in rad/sec
     */
    mockable inline float getRollVelocity() const
    {
        return modm::toRadian(
            static_cast<float>(lastCompleteImuData.rawRollVelocity) /
            tap::communication::sensors::imu::mpu6500::Mpu6500::LSB_D_PER_S_TO_D_PER_S);
    }

    /**
     * @return An unwrapped (not normalized) turret yaw angle, in rad. This object keeps track of
     * the number of revolutions that the attached turret IMU has taken, and the number of
     * revolutions is reset once the IMU is recalibrated or if the turret IMU comes disconnected.
     */
    mockable inline float getRollUnwrapped() const
    {
        return lastCompleteImuData.roll + M_TWOPI * static_cast<float>(rollRevolutions);
    }

    /**
     * @return turret pitch angle in rad, a value normalized between [-pi, pi]
     */
    mockable inline float getPitch() const { return lastCompleteImuData.pitch; }

    /**
     * @return turret pitch angular velocity in rad/sec
     */
    mockable inline float getPitchVelocity() const
    {
        return modm::toRadian(
            static_cast<float>(lastCompleteImuData.rawPitchVelocity) /
            tap::communication::sensors::imu::mpu6500::Mpu6500::LSB_D_PER_S_TO_D_PER_S);
    }

    /**
     * @return An unwrapped (not normalized) turret pitch angle, in rad. This object keeps track of
     * the number of revolutions that the attached turret IMU has taken, and the number of
     * revolutions is reset once the IMU is recalibrated or if the turret IMU comes disconnected.
     */
    mockable inline float getPitchUnwrapped() const
    {
        return lastCompleteImuData.pitch + M_TWOPI * static_cast<float>(pitchRevolutions);
    }

    /**
     * @return turret yaw angle in radians, normalized between [-pi, pi]
     */
    mockable inline float getYaw() const { return lastCompleteImuData.yaw; }

    /**
     * @return turret yaw angular velocity in rad/sec
     */
    mockable inline float getYawVelocity() const
    {
        return modm::toRadian(
            static_cast<float>(lastCompleteImuData.rawYawVelocity) /
            tap::communication::sensors::imu::mpu6500::Mpu6500::LSB_D_PER_S_TO_D_PER_S);
    }

    /**
     * @return An unwrapped (not normalized) turret yaw angle, in rad. This object keeps track of
     * the number of revolutions that the attached turret IMU has taken, and the number of
     * revolutions is reset once the IMU is recalibrated or if the turret IMU comes disconnected.
     */
    mockable inline float getYawUnwrapped() const
    {
        // @todo this is dumb
#ifdef TARGET_SENTRY_HYDRA
        return lastCompleteImuData.yaw + M_TWOPI * static_cast<float>(yawRevolutions) - M_PI;
#else
        return lastCompleteImuData.yaw + M_TWOPI * static_cast<float>(yawRevolutions);
#endif
    }

    mockable inline float getAx() const { return lastCompleteImuData.xAcceleration; }

    mockable inline float getAy() const { return lastCompleteImuData.yAcceleration; }

    mockable inline float getAz() const { return lastCompleteImuData.zAcceleration; }

    mockable inline uint32_t getIMUDataTimestamp() const
    {
        return lastCompleteImuData.turretDataTimestamp;
    }

    inline bool getLimitSwitchDepressed() const final_mockable { return limitSwitchDepressed; }

    mockable inline bool isConnected() const
    {
        return !imuConnectedTimeout.isExpired() && !imuConnectedTimeout.isStopped();
    }

    mockable inline void setOpenHopperCover(bool isOpen)
    {
        txCommandMsgBitmask.update(TxCommandMsgBitmask::OPEN_HOPPER, isOpen);
    }

    mockable inline void setLaserStatus(bool isOn)
    {
        txCommandMsgBitmask.update(TxCommandMsgBitmask::TURN_LASER_ON, isOn);
    }

    mockable inline void sendImuCalibrationRequest()
    {
        txCommandMsgBitmask.set(TxCommandMsgBitmask::RECALIBRATE_IMU);
    }

    mockable void sendData();

private:
    using CanCommListenerFunc = void (TurretMCBCanComm::*)(const modm::can::Message& message);

    static constexpr uint32_t DISCONNECT_TIMEOUT_PERIOD = 100;
    static constexpr float ANGLE_FIXED_POINT_PRECISION = 360.0f / UINT16_MAX;
    static constexpr float CMPS2_TO_MPS2 = 0.01;
    static constexpr uint32_t SEND_MCB_DATA_TIMEOUT = 500;

    class TurretMcbRxHandler : public tap::can::CanRxListener
    {
    public:
        TurretMcbRxHandler(
            tap::Drivers* drivers,
            uint32_t id,
            tap::can::CanBus cB,
            TurretMCBCanComm* msgHandler,
            CanCommListenerFunc funcToCall);
        void processMessage(const modm::can::Message& message) override;

    private:
        TurretMCBCanComm* msgHandler;
        CanCommListenerFunc funcToCall;
    };

    struct AxisMessageData
    {
        int16_t angleFixedPoint;
        int16_t angleAngularVelocityRaw;
        int16_t linearAcceleration;
        uint8_t seq;
    } modm_packed;

    struct ImuData
    {
        float yaw;                     ///< Normalized yaw value, between [-pi, pi]
        int16_t rawYawVelocity;        ///< Raw yaw velocity, in counts per second
        float pitch;                   ///< Normalized pitch value, between [-pi, pi]
        int16_t rawPitchVelocity;      ///< Raw pitch velocity, in counts per second
        float roll;                    ///< Normalized roll value, between [-pi, pi]
        int16_t rawRollVelocity;       ///< Raw roll velocity, in counts per second
        float xAcceleration;           ///< (m/s^2) X-Acceleration
        float yAcceleration;           ///< (m/s^2) Y-Acceleration
        float zAcceleration;           ///< (m/s^2) Z-Acceleration
        uint32_t turretDataTimestamp;  ///< Timestamp that the IMU data was received
        uint8_t seq;                   ///< Sequence number for synchronizing axis messages
    };

    const tap::can::CanBus canBus;

    tap::Drivers* drivers;

    ImuData currProcessingImuData;
    ImuData lastCompleteImuData;

    int yawRevolutions;
    int pitchRevolutions;
    int rollRevolutions;

    TurretMcbRxHandler xAxisMessageHandler;
    TurretMcbRxHandler yAxisMessageHandler;
    TurretMcbRxHandler zAxisMessageHandler;

    TurretMcbRxHandler turretStatusRxHandler;

    TurretMcbRxHandler timeSynchronizationRxHandler;

    tap::arch::MilliTimeout imuConnectedTimeout;

    TxCommandMsgBitmask_t txCommandMsgBitmask;

    tap::arch::PeriodicMilliTimer sendMcbDataTimer;

    int imuMessageReceivedLEDBlinkCounter = 0;

    bool limitSwitchDepressed;

    ImuDataReceivedCallbackFunc imuDataReceivedCallbackFunc = nullptr;

    void handleXAxisMessage(const modm::can::Message& message);

    void handleYAxisMessage(const modm::can::Message& message);

    void handleZAxisMessage(const modm::can::Message& message);

    void handleTurretMessage(const modm::can::Message& message);

    void handleTimeSynchronizationRequest(const modm::can::Message& message);

    /**
     * Updates the passed in revolutionCounter if a revolution increment or decrement has been
     * detected.
     *
     * A revolution increment is detected if the difference between the new and old angle is < -pi,
     * and a decrement is detected if the difference is > pi. Put simply, if the angle measurement
     * jumped unexpectly, it is assumed that a revolution has ocurred.
     *
     * @param[in] newAngle A new angle measurement, in radians.
     * @param[in] prevAngle The old (previous) angle measurement, in radians.
     * @param[out] revolutionCounter Counter to update, either unchanged, incremented, or
     * decremented based on newAngle and prevAngle's state.
     */
    static inline void updateRevolutionCounter(
        const float newAngle,
        const float prevAngle,
        int& revolutionCounter)
    {
        const float angleDiff = newAngle - prevAngle;
        if (angleDiff < -M_PI)
        {
            revolutionCounter++;
        }
        else if (angleDiff > M_PI)
        {
            revolutionCounter--;
        }
    }
};
}  // namespace aruwsrc::can

#endif  // TURRET_MCB_CAN_COMM_HPP_
