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

#include <aruwlib/architecture/timeout.hpp>

#include "dji_serial.hpp"
#include "mock_macros.hpp"

namespace aruwlib
{
class Drivers;
namespace serial
{
/**
 * A class used to communicate with our Xaviers.
 *
 * @note use the static function in Drivers to interact with this class.
 */
class XavierSerial : public DJISerial<>
{
public:
    struct RobotData
    {
        struct
        {
            float ax;  ///< acceleration in \f$\frac{m}{s^2}\f$.
            float ay;
            float az;

            float wx;  ///< gyro values in \f$\frac{degrees}{second}\f$.
            float wy;
            float wz;

            float rol;  ///< Measured in degrees.
            float pit;
            float yaw;
        } IMU;
        struct
        {
            float pitch;
            float yaw;
        } Turret;
        struct
        {
            int16_t rightFrontWheelRPM;
            int16_t leftFrontWheelRPM;
            int16_t leftBackWheeRPM;
            int16_t rightBackWheelRPM;
        } Chassis;
    };

    // AutoAim Data
    struct TurretAimData
    {
        bool hasTarget;      /// Whether or not the xavier has a target.
        float pitch;         /// The pitch angle in degrees, rounded to two decimals.
        float yaw;           /// The yaw angle in degrees, rounded to two decimals.
        uint32_t timestamp;  /// A timestamp in milliseconds.
    };

    XavierSerial(Drivers* drivers);
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
    mockable void sendMessage(const RobotData& robotData);

    /**
     * Start Requesting Xavier to Track Target.
     */
    mockable void beginTargetTracking();
    /**
     * Stop Requesting Xavier to Track Target.
     */
    mockable void stopTargetTracking();

    mockable inline const TurretAimData &getLastAimData() const { return lastAimData; }

    mockable inline bool lastAimDataValid() const { return aimDataValid; }

private:
    enum AutoAimRequestState
    {
        AUTO_AIM_REQUEST_COMPLETE,
        AUTO_AIM_REQUEST_QUEUED,
        AUTO_AIM_REQUEST_SENT,
        AUTO_AIM_REQUEST_RECEIVED,
    };

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

    /// Time in ms since last CV aim data was received before deciding CV is offline.
    static constexpr int16_t TIME_OFFLINE_CV_AIM_DATA_MS = 5000;
    /// Time between each robot id send to CV in milliseconds.
    static constexpr int16_t TIME_BETWEEN_ROBOT_ID_SEND_MS = 5000;
    static constexpr int16_t AUTO_AIM_REQUEST_SEND_PERIOD_MS = 1000;

    // RX message constants for decoding an aim data message. These are zero indexed byte offsets.
    static constexpr uint8_t AIM_DATA_MESSAGE_PITCH_OFFSET = 0;
    static constexpr uint8_t AIM_DATA_MESSAGE_YAW_OFFSET = 4;
    static constexpr uint8_t AIM_DATA_MESSAGE_HAS_TARGET = 8;
    static constexpr uint8_t AIM_DATA_MESSAGE_SIZE = 9;

    /// Used for determining when to send robot id.
    arch::MilliTimeout txRobotIdTimeout;

    /// The most recent auto aim request state.
    struct
    {
        /// `false` if request to stop auto aiming, `true` if request to start auto aiming.
        bool requestType = false;
        AutoAimRequestState currAimState = AUTO_AIM_REQUEST_COMPLETE;
        /// Timer used to reset the aim request if acknowledgement has not been sent by xavier.
        arch::MilliTimeout sendAimRequestTimeout;
    } AutoAimRequest;

    /// The last aim data received from the xavier.
    TurretAimData lastAimData;

    /// Whether or not aim data is up to date.
    bool aimDataValid;

    // CV online variables.
    /// Timer for determining if serial is offline.
    arch::MilliTimeout cvOfflineTimeout;

    /// A flag set to `true` if the timeout is not expired, and `false` otherwise.
    bool isCvOnline;

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

    bool sendRobotMeasurements(const RobotData& robotData);

    /**
     * Packages `robotId` in an acceptable format for the base `DjiSerial` class to interpret
     * and sends the message via `DjiSerial`'s `send` function.
     *
     * @return `true` if sending was a success, `false` otherwise.
     */
    bool sendRobotID();

    bool sendAutoAimRequest();
};
}  // namespace serial
}  // namespace aruwlib

#endif  // XAVIER_SERIAL_HPP_
