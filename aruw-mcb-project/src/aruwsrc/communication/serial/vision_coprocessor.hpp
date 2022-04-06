/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef VISION_COPROCESSOR_HPP_
#define VISION_COPROCESSOR_HPP_

#include <cassert>

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/turret_orientation_interface.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::turret
{
class TurretOrientationInterface;
}

namespace aruwsrc
{
namespace serial
{
/**
 * A class used to communicate with our vision coprocessors. Targets the "Project Otto" vision
 * system (2021-2022).
 *
 * @note use the static function in Drivers to interact with this class.
 */
class VisionCoprocessor : public tap::communication::serial::DJISerial
{
public:
    static_assert(control::turret::NUM_TURRETS > 0, "must have at least 1 turret");

    static constexpr tap::communication::serial::Uart::UartPort VISION_COPROCESSOR_TX_UART_PORT =
        tap::communication::serial::Uart::UartPort::Uart2;

    static constexpr tap::communication::serial::Uart::UartPort VISION_COPROCESSOR_RX_UART_PORT =
        tap::communication::serial::Uart::UartPort::Uart3;

    /**
     * AutoAim data to receive from Jetson.
     */
    struct TurretAimData
    {
        float xPos;          ///< x position of the target (in m).
        float yPos;          ///< y position of the target (in m).
        float zPos;          ///< z position of the target (in m).
        float xVel;          ///< x velocity of the target (in m/s).
        float yVel;          ///< y velocity of the target (in m/s).
        float zVel;          ///< z velocity of the target (in m/s).
        float xAcc;          ///< x acceleration of the target (in m/s^2).
        float yAcc;          ///< y acceleration of the target (in m/s^2).
        float zAcc;          ///< z acceleration of the target (in m/s^2).
        bool hasTarget;      ///< Whether or not the xavier has a target.
        uint32_t timestamp;  ///< Timestamp in microseconds.
    } modm_packed;

    /**
     * Chassis odometry data to send to Jetson.
     */
    struct ChassisOdometryData
    {
        uint32_t timestamp;  ///< timestamp associated with chassis odometry (in us).
        float xPos;          ///< x position of the chassis (in m).
        float yPos;          ///< y position of the chassis (in m).
        float zPos;          ///< z position of the chassis (in m).
        float pitch;         ///< world frame pitch of the chassis (in degrees).
        float yaw;           ///< world frame yaw of the chassis (in degrees).
        float roll;          ///< world frame roll of the chassis (in degrees).
    } modm_packed;

    /**
     * Turret odometry data to send to Jetson.
     */
    struct TurretOdometryData
    {
        uint32_t timestamp;  ///< Timestamp in microseconds, when turret data was computed (in us).
        float pitch;         ///< Pitch angle of turret relative to plane parallel to the ground (in
                             ///< deg).
        float yaw;           ///< Clockwise turret rotation angle between 0 and 360 (in deg).
    } modm_packed;

    struct OdometryData
    {
        ChassisOdometryData chassisOdometry;
        uint8_t numTurrets;
        TurretOdometryData turretOdometry[control::turret::NUM_TURRETS];
    } modm_packed;

    VisionCoprocessor(aruwsrc::Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(VisionCoprocessor);
    mockable ~VisionCoprocessor() = default;

    /**
     * Call this before using the serial line, initializes the uart line
     * and the callback
     */
    mockable void initializeCV();

    /**
     * Handles the types of messages defined above in the RX message handlers section.
     */
    void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;

    /**
     * Cycles through and sends the messages that must be sent to the xavier.
     */
    mockable void sendMessage();

    /**
     * is CV online
     */
    mockable bool isCvOnline();

    mockable inline const TurretAimData& getLastAimData(uint8_t turretID) const
    {
        assert(turretID < control::turret::NUM_TURRETS);
        return lastAimData[turretID];
    }

    mockable inline bool getSomeTurretHasTarget() const
    {
        bool hasTarget = false;
        for (size_t i = 0; i < control::turret::NUM_TURRETS; i++)
        {
            hasTarget |= lastAimData[i].hasTarget;
        }
        return hasTarget;
    }

    mockable inline void attachOdometryInterface(
        tap::algorithms::odometry::Odometry2DInterface* odometryInterface)
    {
        this->odometryInterface = odometryInterface;
    }

    /**
     * Specify the turret orientation for auto-aim to reference based on the target robot.
     */
    mockable inline void attachTurretOrientationInterface(
        aruwsrc::control::turret::TurretOrientationInterface* turretOrientationInterface,
        uint8_t turretID)
    {
        assert(turretID < control::turret::NUM_TURRETS);
        turretOrientationInterfaces[turretID] = turretOrientationInterface;
    }

    mockable void sendShutdownMessage();

    mockable void sendRebootMessage();

    mockable void sendSelectNewTargetMessage();

private:
    enum TxMessageTypes
    {
        CV_MESSAGE_TYPE_ODOMETRY_DATA = 1,
        CV_MESSAGE_TYPE_ROBOT_ID = 6,
        CV_MESSAGE_TYPE_SELECT_NEW_TARGET = 7,
        CV_MESSAGE_TYPE_REBOOT = 8,
        CV_MESSAGE_TYPE_SHUTDOWN = 9,
        CV_MESSAGE_TYPE_TIME_SYNC_RESP = 11,
    };

    enum RxMessageTypes
    {
        CV_MESSAGE_TYPE_TURRET_AIM = 2,
        CV_MESSAGE_TYPE_TIME_SYNC_REQ = 10,
    };

    /// Time in ms since last CV aim data was received before deciding CV is offline.
    static constexpr int16_t TIME_OFFLINE_CV_AIM_DATA_MS = 1000;

    /** Time in ms between sending the robot ID message. */
    static constexpr uint32_t TIME_BTWN_SENDING_ROBOT_ID_MSG = 5'000;

    /** Time in ms between sending the time sync message. */
    static constexpr uint32_t TIME_BTWN_SENDING_TIME_SYNC_DATA = 1'000;

    /// The last aim data received from the xavier.
    TurretAimData lastAimData[control::turret::NUM_TURRETS] = {};

    // CV online variables.
    /// Timer for determining if serial is offline.
    tap::arch::MilliTimeout cvOfflineTimeout;

    tap::algorithms::odometry::Odometry2DInterface* odometryInterface;

    aruwsrc::control::turret::TurretOrientationInterface*
        turretOrientationInterfaces[control::turret::NUM_TURRETS];

    tap::arch::PeriodicMilliTimer sendRobotIdTimeout{TIME_BTWN_SENDING_ROBOT_ID_MSG};

    tap::arch::PeriodicMilliTimer sendTimeSyncTimeout{TIME_BTWN_SENDING_TIME_SYNC_DATA};

    /**
     * Interprets a raw `SerialMessage`'s `data` field to extract yaw, pitch, and other aim
     * data information, and updates the `lastAimData`.
     *
     * @param[in] message the message to be decoded.
     * @param[out] aimData a return parameter through which the decoded message is returned.
     * @return `false` if the message length doesn't match `sizeof(*aimData)`, `true`
     *      otherwise.
     */
    bool decodeToTurretAimData(const ReceivedSerialMessage& message);

    void decodeAndSendTimeSyncMessage(const ReceivedSerialMessage& message);

#ifdef ENV_UNIT_TESTS
public:
#endif

    void sendOdometryData();
    void sendRobotTypeData();
};
}  // namespace serial
}  // namespace aruwsrc

#endif  // VISION_COPROCESSOR_HPP_
