/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/util_macros.hpp"

#include "modm/processing/resumable.hpp"

class VisionCoprocessorTester;

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc
{
namespace serial
{
/**
 * A class used to communicate with our vision coprocessors. Targets the "" vision system
 * (2019-2021).
 *
 * @note use the static function in Drivers to interact with this class.
 */
class VisionCoprocessor : public tap::serial::DJISerial,
                                ::modm::pt::Protothread,
                                modm::Resumable<3>
{
public:
    // AutoAim Data
    struct TurretAimData
    {
        float xPos;         /// x position of the target.
        float yPos;         /// y position of the target.
        float zPos;         /// z position of the target.
        float xVel;         /// x velocity of the target.
        float yVel;         /// y velocity of the target.
        float zVel;         /// z velocity of the target.
        float xAcc;         /// x acceleration of the target.
        float yAcc;         /// y acceleration of the target.
        float zAcc;         /// z acceleration of the target.
        bool hasTarget;      /// Whether or not the xavier has a target.
        uint32_t timestamp;  /// A timestamp in microseconds.
    };

    enum AutoAimRequestState
    {
        AUTO_AIM_REQUEST_COMPLETE = 0,  /// Resting state for sending auto aim request
        AUTO_AIM_REQUEST_QUEUED,  /// Message has been queued but not yet sent requesting auto-aim
        AUTO_AIM_REQUEST_SENT,    /// Message has been sent and a reply is being waited for
    };

    enum TxMessageTypes
    {
        CV_MESSAGE_ODOMETRY_DATA = 1,
        CV_NUM_MESSAGE_TYPES,
    };

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
    void messageReceiveCallback(const SerialMessage& completeMessage) override;

    /**
     * Cycles through and sends the messages that must be sent to the xavier.
     * @note Uses protothread logic
     * @return `false` if the protothread is done (should never happen), or `true` otherwise.
     */
    mockable bool sendMessage();

    /**
     * Start Requesting Xavier to Track Target.
     */
    mockable void beginAutoAim();

    /**
     * Stop Requesting Xavier to Track Target.
     */
    mockable void stopAutoAim();

    mockable inline const TurretAimData& getLastAimData() const { return lastAimData; }

    mockable inline bool lastAimDataValid() const { return aimDataValid; }

private:
    friend class ::VisionCoprocessorTester;

    enum RxMessageTypes
    {
        CV_MESSAGE_TYPE_TURRET_AIM = 0,
    };

    /// Time in ms since last CV aim data was received before deciding CV is offline.
    static constexpr int16_t TIME_OFFLINE_CV_AIM_DATA_MS = 5000;
    /// Time between each robot id send to CV in milliseconds.
    static constexpr int16_t TIME_BETWEEN_ROBOT_ID_SEND_MS = 5000;
    /// Time between auto aim requests (which are resent until aim data is sent from the xavier)
    static constexpr int16_t AUTO_AIM_REQUEST_SEND_PERIOD_MS = 1000;
    /// Precision of the floating point data sent to the Xavier.
    static constexpr float FIXED_POINT_PRECISION = 0.01f;

    // RX message constants for decoding an aim data message. These are zero indexed byte offsets.
    /// Offset for x position
    static constexpr uint8_t AIM_DATA_MESSAGE_X_POSITION_OFFSET = 0;
    /// Offset for y position
    static constexpr uint8_t AIM_DATA_MESSAGE_Y_POSITION_OFFSET = sizeof(uint32_t);
    /// Offset for z position
    static constexpr uint8_t AIM_DATA_MESSAGE_Z_POSITION_OFFSET = 2 * sizeof(uint32_t);
    /// Offset for x velocity
    static constexpr uint8_t AIM_DATA_MESSAGE_X_VELOCITY_OFFSET = 3 * sizeof(uint32_t);
    /// Offset for y velocity
    static constexpr uint8_t AIM_DATA_MESSAGE_Y_VELOCITY_OFFSET = 4 * sizeof(uint32_t);
    /// Offset for z velocity
    static constexpr uint8_t AIM_DATA_MESSAGE_Z_VELOCITY_OFFSET = 5 * sizeof(uint32_t);
    /// Offset for x acceleration
    static constexpr uint8_t AIM_DATA_MESSAGE_X_ACCELERATION_OFFSET = 6 * sizeof(uint32_t);
    /// Offset for y acceleration
    static constexpr uint8_t AIM_DATA_MESSAGE_Y_ACCELERATION_OFFSET = 7 * sizeof(uint32_t);
    /// Offset for z acceleration
    static constexpr uint8_t AIM_DATA_MESSAGE_Z_ACCELERATION_OFFSET = 8 * sizeof(uint32_t);
    /// Offset for whether or not cv has data
    static constexpr uint8_t AIM_DATA_MESSAGE_HAS_TARGET_OFFSET = 8 * sizeof(uint32_t) + sizeof(uint8_t);
    /// Offset for timestamp in microseconds
    static constexpr uint8_t AIM_DATA_MESSAGE_TIMESTAMP_MICROS_OFFSET = 9 * sizeof(uint32_t) + sizeof(uint8_t);
    /// Size of entire message
    static constexpr uint8_t AIM_DATA_MESSAGE_SIZE = 10 * sizeof(uint32_t) + sizeof(uint8_t);
    
    // TX message constants for encoding odometry data. These are zero indexed byte offsets.
    static constexpr uint8_t ODOMETRY_DATA_MESSAGE_X_POSITION_OFFSET = 0;
    static constexpr uint8_t ODOMETRY_DATA_MESSAGE_Y_POSITION_OFFSET = sizeof(uint32_t);
    static constexpr uint8_t ODOMETRY_DATA_MESSAGE_X_POSITION_OFFSET = 2 * sizeof(uint32_t);
    static constexpr uint8_t ODOMETRY_DATA_MESSAGE_TURRET_PITCH_OFFSET = 3 * sizeof(uint32_t);
    static constexpr uint8_t ODOMETRY_DATA_MESSAGE_TURRET_YAW_OFFSET = 4 * sizeof(uint32_t);
    static constexpr uint8_t ODOMETRY_DATA_MESSAGE_TIMESTAMP_MICROS_OFFSET = 5 * sizeof(uint32_t);
    static constexpr uint8_t ODOMETRY_DATA_MESSAGE_SIZE = 6 * sizeof(uint32_t);

    /// Used for determining when to send robot id.
    tap::arch::PeriodicMilliTimer txRobotIdTimeout;

    /// The most recent auto aim request state.
    struct
    {
        /// `false` if request to stop auto aiming, `true` if request to start auto aiming.
        bool autoAimRequest = false;
        AutoAimRequestState currAimState = AUTO_AIM_REQUEST_COMPLETE;
        /// Timer used to reset the aim request if acknowledgement has not been sent by xavier.
        tap::arch::MilliTimeout sendAimRequestTimeout;
    } AutoAimRequest;

    /// The last aim data received from the xavier.
    TurretAimData lastAimData;

    /// Whether or not aim data is up to date.
    bool aimDataValid;

    // CV online variables.
    /// Timer for determining if serial is offline.
    tap::arch::MilliTimeout cvOfflineTimeout;

    /// A flag set to `true` if the timeout is not expired, and `false` otherwise.
    bool isCvOnline;

    const aruwsrc::can::TurretMCBCanComm* turretMCBCanComm;

    /**
     * TODO: update this specification
     * Interprets a raw `SerialMessage`'s `data` field to extract yaw, pitch, and other aim
     * data information, and updates the `lastAimData`.
     *
     * @param[in] message the message to be decoded.
     * @param[out] aimData a return parameter through which the decoded message is returned.
     * @return `false` if the message length doesn't match `AIM_DATA_MESSAGE_SIZE`, `true`
     *      otherwise.
     */
    static bool decodeToTurretAimData(const SerialMessage& message, TurretAimData* aimData);

#ifdef ENV_UNIT_TESTS
public:
#endif

    modm::ResumableResult<bool> sendOdometryData();

    modm::ResumableResult<bool> sendAutoAimRequest();
};
}  // namespace serial
}  // namespace aruwsrc

#endif  // VISION_COPROCESSOR_HPP_
