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

#include "tap/communication/serial/dji_serial.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::can
{
class TurretMCBCanComm;
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
class VisionCoprocessor : public tap::serial::DJISerial
{
public:
    /**
     * AutoAim data to receive from Jetson.
     */
    struct TurretAimData
    {
        float xPos;          /// x position of the target.
        float yPos;          /// y position of the target.
        float zPos;          /// z position of the target.
        float xVel;          /// x velocity of the target.
        float yVel;          /// y velocity of the target.
        float zVel;          /// z velocity of the target.
        float xAcc;          /// x acceleration of the target.
        float yAcc;          /// y acceleration of the target.
        float zAcc;          /// z acceleration of the target.
        bool hasTarget;      /// Whether or not the xavier has a target.
        uint32_t timestamp;  /// Timestamp in microseconds.
    } modm_packed;

    /**
     * Odometry data to send to Jetson.
     */
    struct OdometryData
    {
        float chassisX;      /// x position of the chassis.
        float chassisY;      /// y position of the chassis.
        float chassisZ;      /// z position of the chassis.
        float turretPitch;   /// Pitch angle of turret relative to plane parallel to the ground.
        float turretYaw;     /// Clockwise turret rotation angle between 0 and 360.
        uint32_t timestamp;  /// Timestamp in microseconds.
    } modm_packed;

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
     */
    mockable void sendMessage();

    mockable inline const TurretAimData& getLastAimData() const { return lastAimData; }

private:
    enum RxMessageTypes
    {
        CV_MESSAGE_TYPE_TURRET_AIM = 0,
    };

    /// The last aim data received from the xavier.
    TurretAimData lastAimData;

    const aruwsrc::can::TurretMCBCanComm* turretMCBCanComm;

    /**
     * Interprets a raw `SerialMessage`'s `data` field to extract yaw, pitch, and other aim
     * data information, and updates the `lastAimData`.
     *
     * @param[in] message the message to be decoded.
     * @param[out] aimData a return parameter through which the decoded message is returned.
     * @return `false` if the message length doesn't match `sizeof(*aimData)`, `true`
     *      otherwise.
     */
    static bool decodeToTurretAimData(const SerialMessage& message, TurretAimData* aimData);

#ifdef ENV_UNIT_TESTS
public:
#endif

    void sendOdometryData();
};
}  // namespace serial
}  // namespace aruwsrc

#endif  // VISION_COPROCESSOR_HPP_
