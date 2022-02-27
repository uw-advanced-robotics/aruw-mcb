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

#ifndef TURRET_MCB_CAN_COMM_
#define TURRET_MCB_CAN_COMM_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/communication/sensors/mpu6500/mpu6500.hpp"

#include "modm/architecture/interface/register.hpp"

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
 */
class TurretMCBCanComm
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

    TurretMCBCanComm(aruwsrc::Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(TurretMCBCanComm);

    mockable void init();

    mockable inline void attachImuDataReceivedCallback(ImuDataReceivedCallbackFunc func)
    {
        imuDataReceivedCallbackFunc = func;
    }

    /** @return turret pitch angle in deg */
    mockable inline float getPitch() const { return lastCompleteImuData.pitch; }
    /** @return turret pitch angular velocity in deg/sec */
    mockable inline float getPitchVelocity() const
    {
        return static_cast<float>(lastCompleteImuData.rawPitchVelocity) /
               tap::sensors::Mpu6500::LSB_D_PER_S_TO_D_PER_S;
    }

    /** @return turret yaw angle in degrees */
    mockable inline float getYaw() const { return lastCompleteImuData.yaw; }
    /** @return turret yaw angular velocity in deg/sec */
    mockable inline float getYawVelocity() const
    {
        return static_cast<float>(lastCompleteImuData.rawYawVelocity) /
               tap::sensors::Mpu6500::LSB_D_PER_S_TO_D_PER_S;
    }

    mockable inline bool getLimitSwitchDepressed() const { return limitSwitchDepressed; }

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

    mockable inline uint32_t getIMUDataTimestamp() const
    {
        return lastCompleteImuData.turretDataTimestamp;
    }

private:
    using CanCommListenerFunc = void (TurretMCBCanComm::*)(const modm::can::Message& message);

    enum CanIDs
    {
        SYNC_RX_CAN_ID = 0x1f9,
        SYNC_TX_CAN_ID = 0x1fa,
        TURRET_STATUS_RX_CAN_ID = 0x1fb,
        PITCH_RX_CAN_ID = 0x1fc,
        YAW_RX_CAN_ID = 0x1fd,
        TURRET_MCB_TX_CAN_ID = 0x1fe,
    };

    static constexpr tap::can::CanBus TURRET_MCB_CAN_BUS = tap::can::CanBus::CAN_BUS1;
    static constexpr uint32_t DISCONNECT_TIMEOUT_PERIOD = 100;
    static constexpr float ANGLE_FIXED_POINT_PRECISION = 360.0f / UINT16_MAX;
    static constexpr uint32_t SEND_MCB_DATA_TIMEOUT = 500;

    class TurretMcbRxHandler : public tap::can::CanRxListener
    {
    public:
        TurretMcbRxHandler(
            aruwsrc::Drivers* drivers,
            uint32_t id,
            tap::can::CanBus cB,
            TurretMCBCanComm* msgHandler,
            CanCommListenerFunc funcToCall);
        void processMessage(const modm::can::Message& message) override;

    private:
        TurretMCBCanComm* msgHandler;
        CanCommListenerFunc funcToCall;
    };

    struct AngleMessageData
    {
        int16_t angleFixedPoint;
        int16_t angleAngularVelocityRaw;
        uint8_t seq;
        uint16_t timestamp;
    } modm_packed;

    struct ImuData
    {
        float yaw;
        int16_t rawYawVelocity;
        float pitch;
        int16_t rawPitchVelocity;
        uint32_t turretDataTimestamp;
        uint8_t seq;
    };

    aruwsrc::Drivers* drivers;

    ImuData currProcessingImuData;
    ImuData lastCompleteImuData;

    TurretMcbRxHandler yawAngleGyroMessageHandler;

    TurretMcbRxHandler pitchAngleGyroMessageHandler;

    TurretMcbRxHandler turretStatusRxHandler;

    TurretMcbRxHandler timeSynchronizationRxHandler;

    tap::arch::MilliTimeout imuConnectedTimeout;

    TxCommandMsgBitmask_t txCommandMsgBitmask;

    tap::arch::PeriodicMilliTimer sendMcbDataTimer;

    int imuMessageReceivedLEDBlinkCounter = 0;

    bool limitSwitchDepressed;

    ImuDataReceivedCallbackFunc imuDataReceivedCallbackFunc = nullptr;

    void handleYawAngleGyroMessage(const modm::can::Message& message);

    void handlePitchAngleGyroMessage(const modm::can::Message& message);

    void handleTurretMessage(const modm::can::Message& message);

    void handleTimeSynchronizationRequest(const modm::can::Message& message);
};
}  // namespace aruwsrc::can

#endif  // TURRET_MCB_CAN_COMM_
