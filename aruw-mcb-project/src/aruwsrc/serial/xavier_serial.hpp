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

#ifndef XAVIER_SERIAL_HPP_
#define XAVIER_SERIAL_HPP_

#include <aruwlib/architecture/periodic_timer.hpp>
#include <aruwlib/architecture/timeout.hpp>
#include <aruwlib/communication/serial/dji_serial.hpp>

#include "util_macros.hpp"

class XavierSerialTester;

namespace aruwlib
{
class Drivers;
}

namespace aruwsrc
{
namespace turret
{
class TurretSubsystem;
}

namespace chassis
{
class ChassisSubsystem;
}

namespace serial
{
/**
 * A class used to communicate with our Xaviers.
 *
 * @note use the static function in Drivers to interact with this class.
 */
class XavierSerial : public aruwlib::serial::DJISerial<true>
{
public:
    // AutoAim Data
    struct TurretAimData
    {
        bool hasTarget;      /// Whether or not the xavier has a target.
        float pitch;         /// The pitch angle in degrees, rounded to two decimals.
        float yaw;           /// The yaw angle in degrees, rounded to two decimals.
        uint32_t timestamp;  /// A timestamp in milliseconds.
    };

    enum AutoAimRequestState
    {
        AUTO_AIM_REQUEST_COMPLETE,
        AUTO_AIM_REQUEST_QUEUED,
        AUTO_AIM_REQUEST_SENT,
    };

    XavierSerial(
        aruwlib::Drivers* drivers,
        const turret::TurretSubsystem* turretSub,
        const chassis::ChassisSubsystem* chassisSub);
    XavierSerial(const XavierSerial&) = delete;
    XavierSerial& operator=(const XavierSerial&) = delete;
    mockable ~XavierSerial() = default;

    /**
     * Call this before using the serial line, initializes the uart line
     * and the callback
     */
    mockable void initializeCV();

    /**
     * Handles the types of messages defined above in the RX message handlers section.
     */
    void messageReceiveCallback(const SerialMessage& completeMessage) override;

    /**
     * Cycles through the messages that must be sent to the xavier.
     */
    mockable void sendMessage();

    /**
     * Start Requesting Xavier to Track Target.
     */
    mockable void beginAutoAim();

    /**
     * Stop Requesting Xavier to Track Target.
     */
    mockable void stopAutoAim();

    mockable_inline const TurretAimData& getLastAimData() const { return lastAimData; }

    mockable_inline bool lastAimDataValid() const { return aimDataValid; }

    mockable_inline void attachTurret(turret::TurretSubsystem* turret) { turretSub = turret; }
    mockable_inline void attachChassis(chassis::ChassisSubsystem* chassis) { chassisSub = chassis; }

private:
    friend class ::XavierSerialTester;

    enum TxMessageTypes
    {
        CV_MESSAGE_TYPE_ROBOT_DATA = 0,
        CV_MESSAGE_TYPE_ROBOT_ID,
        CV_MESSAGE_TYPE_AUTO_AIM_REQUEST,
        CV_MESSAGE_TYPE_SIZE,
    };

    enum RxMessageTypes
    {
        CV_MESSAGE_TYPE_TURRET_AIM = 0,
        CV_MESSAGE_TYPE_TRACKING_REQUEST_ACKN
    };

    enum TxMessageState
    {
        FAIL,
        SUCCESS,
        DID_NOT_SEND,
    };

    /// Time in ms since last CV aim data was received before deciding CV is offline.
    static constexpr int16_t TIME_OFFLINE_CV_AIM_DATA_MS = 5000;
    /// Time between each robot id send to CV in milliseconds.
    static constexpr int16_t TIME_BETWEEN_ROBOT_ID_SEND_MS = 5000;
    static constexpr int16_t AUTO_AIM_REQUEST_SEND_PERIOD_MS = 1000;
    static constexpr float FIXED_POINT_PRECISION = 0.001f;

    // RX message constants for decoding an aim data message. These are zero indexed byte offsets.
    static constexpr uint8_t AIM_DATA_MESSAGE_PITCH_OFFSET = 0;
    static constexpr uint8_t AIM_DATA_MESSAGE_YAW_OFFSET = 4;
    static constexpr uint8_t AIM_DATA_MESSAGE_HAS_TARGET = 8;
    static constexpr uint8_t AIM_DATA_MESSAGE_SIZE = 9;

    // TX message constants for encoding robot data. These are zero indexed byte offsets.
    static constexpr uint8_t CHASSIS_DATA_OFFSET = 0;
    static constexpr uint8_t TURRET_DATA_OFFSET = 4 * sizeof(uint16_t);
    static constexpr uint8_t IMU_DATA_OFFSET = TURRET_DATA_OFFSET + 2 * sizeof(int32_t);
    static constexpr uint8_t ROBOT_DATA_MSG_SIZE = IMU_DATA_OFFSET + 9 * sizeof(int32_t);

    /// Message that we are currently sending
    TxMessageTypes currTxMessageType;

    /// Used for determining when to send robot id.
    aruwlib::arch::PeriodicMilliTimer txRobotIdTimeout;

    /// The most recent auto aim request state.
    struct
    {
        /// `false` if request to stop auto aiming, `true` if request to start auto aiming.
        bool requestType = false;
        AutoAimRequestState currAimState = AUTO_AIM_REQUEST_COMPLETE;
        /// Timer used to reset the aim request if acknowledgement has not been sent by xavier.
        aruwlib::arch::MilliTimeout sendAimRequestTimeout;
    } AutoAimRequest;

    /// The last aim data received from the xavier.
    TurretAimData lastAimData;

    /// Whether or not aim data is up to date.
    bool aimDataValid;

    // CV online variables.
    /// Timer for determining if serial is offline.
    aruwlib::arch::MilliTimeout cvOfflineTimeout;

    /// A flag set to `true` if the timeout is not expired, and `false` otherwise.
    bool isCvOnline;

    const turret::TurretSubsystem* turretSub;
    const chassis::ChassisSubsystem* chassisSub;

    /**
     * Flag indicating if sending robot id succeeded or not, set to false initially so
     * robot id will be sent right away
     */
    bool robotIdSendSucceeded = false;

    /**
     * Interprets a raw `SerialMessage`'s `data` field to extract yaw, pitch, and other aim
     * data information.
     *
     * @param[in] message the message to be decoded.
     * @param[out] aimData a return parameter through which the decoded message is returned.
     * @return `false` if the message length doesn't match `AIM_DATA_MESSAGE_SIZE`, `true`
     *      otherwise.
     */
    bool decodeToTurrentAimData(const SerialMessage& message, TurretAimData* aimData);

    TxMessageState sendRobotMeasurements();

    /**
     * Packages `robotId` in an acceptable format for the base `DjiSerial` class to interpret
     * and sends the message via `DjiSerial`'s `send` function.
     *
     * @return `SUCCESS` if sending was a success, `FAIL` if send failed, `DID_NOT_SEND` otherwise.
     */
    TxMessageState sendRobotID();

    TxMessageState sendAutoAimRequest();
};
}  // namespace serial
}  // namespace aruwsrc

#endif  // XAVIER_SERIAL_HPP_
