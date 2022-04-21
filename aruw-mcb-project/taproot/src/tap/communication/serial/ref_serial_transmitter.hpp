/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_REF_SERIAL_TRANSMITTER_HPP_
#define TAPROOT_REF_SERIAL_TRANSMITTER_HPP_

#include "tap/architecture/timeout.hpp"
#include "tap/communication/serial/ref_serial_constants.hpp"
#include "tap/util_macros.hpp"

#include "modm/processing/resumable.hpp"

#include "dji_serial.hpp"
#include "ref_serial_data.hpp"

namespace tap
{
class Drivers;
}

namespace tap::communication::serial
{
/**
 * Encapsulates logic for sending messages to the referee system. This includes sending
 * robot-to-robot communication messages and UI drawing instructions to the RoboMaster client.
 *
 * The instance member functions of this class that return a `modm::ResumableResult<void>` must be
 * used in a protothread or resumable function. In other words, member functions such as
 * `sendGraphic` must be in a call of `PT_CALL` or `RF_CALL`.
 *
 * This transmitter allows the user to send data to the referee system from multiple protothreads.
 * An instance of the ref serial transmitter should be instantiated for each protothread. If unique
 * instances are not used, behavior is undefined.
 */
class RefSerialTransmitter : public RefSerialData, public modm::Resumable<7>
{
public:
    RefSerialTransmitter(Drivers* drivers);

    /**
     * Configures the `graphicData` with all data generic to the type of graphic being configured.
     *
     * For sending graphics, the general schema is to create a `Graphic<n>Message` struct, configure
     * the individual `GraphicData` structs in the graphic message using the `configGraphicGenerics`
     * and then `config<Line|Rectangle|Circle|etc.>` functions. Finally, send the graphic message
     * using `sendGraphic`.
     *
     * For example, to configure and send a line graphic (`refSerial` is a pointer to the global
     * `RefSerial` object):
     *
     * ```
     * Graphic1Message msg;
     * refSerial->configGraphicGenerics(&msg.graphicData, "\x00\x00\x01", RefSerial::GRAPHIC_ADD,1,
     * YELLOW); refSerial->configLine(4, 100, 100, 200, 200, &msg.graphicData);
     * refSerial->sendGraphic(&msg);
     * ```
     *
     * @param[out] graphicData The structure where generic data will be stored.
     * @param[in] name The name of the graphic.
     * @param[in] operation The graphic operation to be done (add/remove, etc).
     * @param[in] layer The graphic layer the graphic will be located at. Must be between 0-9
     * @param[in] color The color of the graphic.
     */
    static void configGraphicGenerics(
        Tx::GraphicData* graphicData,
        const uint8_t* name,
        Tx::GraphicOperation operation,
        uint8_t layer,
        Tx::GraphicColor color);
    /**
     * Configures `sharedData` with a line graphic whose parameters are specified.
     *
     * @param[out] sharedData The graphic data struct to configure.
     */
    static void configLine(
        uint16_t width,
        uint16_t startX,
        uint16_t startY,
        uint16_t endX,
        uint16_t endY,
        Tx::GraphicData* sharedData);
    /**
     * Configures `sharedData` with a rectangle graphic whose parameters are specified.
     *
     * @param[out] sharedData The graphic data struct to configure.
     */
    static void configRectangle(
        uint16_t width,
        uint16_t startX,
        uint16_t startY,
        uint16_t endX,
        uint16_t endY,
        Tx::GraphicData* sharedData);
    /**
     * Configures `sharedData` with a circle graphic whose parameters are specified.
     *
     * @param[out] sharedData The graphic data struct to configure.
     */
    static void configCircle(
        uint16_t width,
        uint16_t centerX,
        uint16_t centerY,
        uint16_t radius,
        Tx::GraphicData* sharedData);
    /**
     * Configures `sharedData` with an ellipse graphic whose parameters are specified.
     *
     * @param[out] sharedData The graphic data struct to configure.
     */
    static void configEllipse(
        uint16_t width,
        uint16_t centerX,
        uint16_t centerY,
        uint16_t xLen,
        uint16_t yLen,
        Tx::GraphicData* sharedData);
    /**
     * Configures `sharedData` with an arc graphic whose parameters are specified.
     *
     * @param[out] sharedData The graphic data struct to configure.
     */
    static void configArc(
        uint16_t startAngle,
        uint16_t endAngle,
        uint16_t width,
        uint16_t centerX,
        uint16_t centerY,
        uint16_t xLen,
        uint16_t yLen,
        Tx::GraphicData* sharedData);
    /**
     * Configures `sharedData` with a floating point number.
     *
     * Recommended font size and line width ratio is 10:1.
     *
     * @note This function doesn't work because of known issues in the referee system
     *      server.
     *
     * @param[out] sharedData The graphic data struct to configure.
     */
    static void configFloatingNumber(
        uint16_t fontSize,
        uint16_t decimalPrecision,
        uint16_t width,
        uint16_t startX,
        uint16_t startY,
        float value,
        Tx::GraphicData* sharedData);
    /**
     * Configures `sharedData` with an integer.
     *
     * Recommended font size and line width ratio is 10:1.
     *
     * @note This function doesn't display negative numbers properly because of known
     *      issues in the referee system server.
     *
     * @param[out] sharedData The graphic data struct to configure.
     */
    static void configInteger(
        uint16_t fontSize,
        uint16_t width,
        uint16_t startX,
        uint16_t startY,
        int32_t value,
        Tx::GraphicData* sharedData);
    /**
     * Configures a character message in the passed in `GraphicCharacterMessage`.
     *
     * Recommended font size and line width ratio is 10:1.
     *
     * @param[out] sharedData The message to configure.
     */
    static void configCharacterMsg(
        uint16_t fontSize,
        uint16_t width,
        uint16_t startX,
        uint16_t startY,
        const char* dataToPrint,
        Tx::GraphicCharacterMessage* sharedData);

    /**
     * Properly constructs the frame header and places it in the passed in `header`.
     *
     * @param[out] header The frame header struct to store the constructed frame header.
     * @param[in] msgLen The length of the message. This includes only the length of the data
     *      and not the length of the cmdId or frame tail.
     */
    static void configFrameHeader(DJISerial::FrameHeader* header, uint16_t msgLen);
    static void configInteractiveHeader(
        Tx::InteractiveHeader* header,
        uint16_t cmdId,
        RobotId senderId,
        uint16_t receiverId);

    /**
     * Deletes an entire graphic layer or the entire screen
     *
     * @param[in] graphicOperation Whether to delete a single layer or the entire screen.
     * @param[in] graphicLayer The layer to remove. Must be between 0-9
     */
    mockable modm::ResumableResult<void> deleteGraphicLayer(
        Tx::DeleteGraphicOperation graphicOperation,
        uint8_t graphicLayer);

    /**
     * This function and the ones below all configure the message header and sends the specified
     * message struct to the referee system unless `configMsgHeader` or `sendMsg` are false.
     *
     * @param[in] graphicMsg The graphic message to send. Note that this struct is updated
     *      with header information in this function.
     * @param[in] configMsgHeader Whether or not to update the `graphicMsg`'s header information.
     * @param[in] sendMsg Whether or not to send the message.
     */
    ///@{
    mockable modm::ResumableResult<void> sendGraphic(
        Tx::Graphic1Message* graphicMsg,
        bool configMsgHeader = true,
        bool sendMsg = true);
    mockable modm::ResumableResult<void> sendGraphic(
        Tx::Graphic2Message* graphicMsg,
        bool configMsgHeader = true,
        bool sendMsg = true);
    mockable modm::ResumableResult<void> sendGraphic(
        Tx::Graphic5Message* graphicMsg,
        bool configMsgHeader = true,
        bool sendMsg = true);
    mockable modm::ResumableResult<void> sendGraphic(
        Tx::Graphic7Message* graphicMsg,
        bool configMsgHeader = true,
        bool sendMsg = true);
    mockable modm::ResumableResult<void> sendGraphic(
        Tx::GraphicCharacterMessage* graphicMsg,
        bool configMsgHeader = true,
        bool sendMsg = true);
    ///@}

    mockable modm::ResumableResult<void> sendRobotToRobotMsg(
        Tx::RobotToRobotMessage* robotToRobotMsg,
        uint16_t msgId,
        RobotId receiverId,
        uint16_t msgLen);

private:
    tap::Drivers* drivers;
    tap::arch::MilliTimeout delayTimer;
    Tx::DeleteGraphicLayerMessage deleteGraphicLayerMessage;
};
}  // namespace tap::communication::serial

#endif  // TAPROOT_REF_SERIAL_TRANSMITTER_HPP_
