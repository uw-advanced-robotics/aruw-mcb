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
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"

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
    TurretMCBCanComm(aruwsrc::Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(TurretMCBCanComm);

    mockable void init();

    /** @return turret pitch angle in deg */
    mockable inline float getPitch() const { return pitch; }
    /** @return turret pitch angular velocity in deg/sec */
    mockable inline float getPitchVelocity() const
    {
        return static_cast<float>(rawPitchVelocity) / tap::communication::sensors::imu::mpu6500::Mpu6500::LSB_D_PER_S_TO_D_PER_S;
    }

    /** @return turret yaw angle in degrees */
    mockable inline float getYaw() const { return yaw; }
    /** @return turret yaw angular velocity in deg/sec */
    mockable inline float getYawVelocity() const
    {
        return static_cast<float>(rawYawVelocity) / tap::communication::sensors::imu::mpu6500::Mpu6500::LSB_D_PER_S_TO_D_PER_S;
    }

    mockable inline float isConnected() const
    {
        return !imuConnectedTimeout.isExpired() && !imuConnectedTimeout.isStopped();
    }

    void setOpenHopperCover(bool isOpen) { openHopperCover = isOpen; }

    mockable void sendImuCalibrationRequest() { calibrateImu = true; }

    void sendData();

private:
    using ImuRxListenerFunc = void (TurretMCBCanComm::*)(const modm::can::Message& message);

    static constexpr uint32_t ANGLE_GYRO_RX_CAN_ID = 0x1fd;
    static constexpr uint32_t TURRET_MCB_TX_CAN_ID = 0x1fe;
    static constexpr tap::can::CanBus IMU_MSG_CAN_BUS = tap::can::CanBus::CAN_BUS1;
    static constexpr uint32_t DISCONNECT_TIMEOUT_PERIOD = 100;
    static constexpr float ANGLE_FIXED_POINT_PRECISION = 360.0f / UINT16_MAX;
    static constexpr uint32_t SEND_MCB_DATA_TIMEOUT = 500;

    class ImuRxHandler : public tap::can::CanRxListener
    {
    public:
        ImuRxHandler(
            aruwsrc::Drivers* drivers,
            uint32_t id,
            tap::can::CanBus cB,
            TurretMCBCanComm* msgHandler,
            ImuRxListenerFunc funcToCall);
        void processMessage(const modm::can::Message& message) override;

    private:
        TurretMCBCanComm* msgHandler;
        ImuRxListenerFunc funcToCall;
    };

    aruwsrc::Drivers* drivers;

    float yaw;
    int16_t rawYawVelocity;

    float pitch;
    int16_t rawPitchVelocity;

    ImuRxHandler angleGyroMessageHandler;

    tap::arch::MilliTimeout imuConnectedTimeout;

    bool openHopperCover;

    bool calibrateImu;

    tap::arch::PeriodicMilliTimer sendMcbDataTimer;

    int imuMessageReceivedLEDBlinkCounter = 0;

    void handleAngleGyroMessage(const modm::can::Message& message);
};
}  // namespace aruwsrc::can

#endif  // TURRET_MCB_CAN_COMM_
