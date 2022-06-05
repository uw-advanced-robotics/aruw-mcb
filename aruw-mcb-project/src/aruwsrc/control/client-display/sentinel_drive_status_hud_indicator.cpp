/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "sentinel_drive_status_hud_indicator.hpp"

#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "aruwsrc/control/client-display/boolean_hud_indicators.hpp"
#include "aruwsrc/drivers.hpp"

using namespace tap::communication::serial;
using namespace aruwsrc::control::client_display;

namespace aruwsrc::control::client_display
{
SentinelDriveStatusHudIndicator::SentinelDriveStatusHudIndicator(
    aruwsrc::Drivers &drivers,
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
    aruwsrc::control::sentinel::drive::SentinelAutoDriveComprisedCommand &command)
    : aruwsrc::control::client_display::HudIndicator(refSerialTransmitter),
      drivers(drivers),
      command(command)
{
}

modm::ResumableResult<bool> SentinelDriveStatusHudIndicator::sendInitialGraphics()
{
    RF_BEGIN(0)

    for (i = 0; i < size_t(RobotHudsToUpdate::NUM_ROBOT_HUDS); i++)
    {
        this->indicatorCircle[i].graphicData.operation = Tx::GraphicOperation::GRAPHIC_ADD;

        this->configFrameIdForSpecificRobot(
            &this->indicatorCircle[i],
            this->indicatorCircle[i].interactiveHeader.receiverId);

        RF_CALL(this->refSerialTransmitter.sendGraphic(&this->indicatorText[i], false, true));
        RF_CALL(
            this->refSerialTransmitter.sendGraphic(&this->indicatorCircleOutline[i], false, true));
        RF_CALL(this->refSerialTransmitter.sendGraphic(&this->indicatorCircle[i], false, true));

        this->indicatorCircle[i].graphicData.operation = Tx::GraphicOperation::GRAPHIC_MODIFY;

        this->configFrameIdForSpecificRobot(
            &this->indicatorCircle[i],
            this->indicatorCircle[i].interactiveHeader.receiverId);
    }

    RF_END();
}

modm::ResumableResult<bool> SentinelDriveStatusHudIndicator::update()
{
    RF_BEGIN(1);

    if (resendBaseGraphicDataTimer.execute())
    {
        RF_CALL(sendInitialGraphics());
    }
    else if (this->getChassisMovementStatus() != this->prevChassisMovementStatus)
    {
        this->prevChassisMovementStatus = this->getChassisMovementStatus();

        for (i = 0; i < size_t(RobotHudsToUpdate::NUM_ROBOT_HUDS); i++)
        {
            // update color and re-configure frame header

            this->indicatorCircle[i].graphicData.color = static_cast<uint8_t>(
                this->prevChassisMovementStatus ? SENTINEL_DRIVE_MOVING
                                                : SENTINEL_DRIVE_NOT_MOVING);

            this->configFrameIdForSpecificRobot(
                &this->indicatorCircle[i],
                this->indicatorCircle[i].interactiveHeader.receiverId);

            RF_CALL(this->refSerialTransmitter.sendGraphic(&this->indicatorCircle[i], false, true));
        }
    }

    RF_END();
}

static uint16_t getRobotClientID(RefSerialTransmitter::RobotId robotId)
{
    return 0x100 + static_cast<uint16_t>(robotId);
}

void SentinelDriveStatusHudIndicator::initialize()
{
    for (size_t i = 0; i < size_t(RobotHudsToUpdate::NUM_ROBOT_HUDS); i++)
    {
        uint8_t booleanHudIndicatorName[3] = {};
        // Hack to avoid conflicting hud indicators when sending to other robot HUDs
        booleanHudIndicatorName[0] = 0x0f;
        booleanHudIndicatorName[1] = 0x0f;
        booleanHudIndicatorName[2] = 0x0f;

        // config inner circle

        RefSerialTransmitter::configGraphicGenerics(
            &this->indicatorCircle[i].graphicData,
            booleanHudIndicatorName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            SENTINEL_DRIVE_NOT_MOVING);

        RefSerialTransmitter::configCircle(
            BooleanHudIndicators::BOOLEAN_HUD_INDICATOR_WIDTH,
            BooleanHudIndicators::BOOLEAN_HUD_INDICATOR_LIST_CENTER_X,
            SENTINEL_DRIVE_STATUS_Y,
            BooleanHudIndicators::BOOLEAN_HUD_INDICATOR_RADIUS,
            &this->indicatorCircle[i].graphicData);

        // config outer circle

        booleanHudIndicatorName[2]++;

        RefSerialTransmitter::configGraphicGenerics(
            &this->indicatorCircleOutline[i].graphicData,
            booleanHudIndicatorName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            BooleanHudIndicators::BOOLEAN_HUD_INDICATOR_OUTLINE_COLOR);

        RefSerialTransmitter::configCircle(
            BooleanHudIndicators::BOOLEAN_HUD_INDICATOR_OUTLINE_WIDTH,
            BooleanHudIndicators::BOOLEAN_HUD_INDICATOR_LIST_CENTER_X,
            SENTINEL_DRIVE_STATUS_Y,
            BooleanHudIndicators::BOOLEAN_HUD_INDICATOR_OUTLINE_RADIUS,
            &this->indicatorCircleOutline[i].graphicData);

        // config text

        booleanHudIndicatorName[2]++;

        RefSerialTransmitter::configGraphicGenerics(
            &this->indicatorText[i].graphicData,
            booleanHudIndicatorName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            BooleanHudIndicators::BOOLEAN_HUD_INDICATOR_LABEL_COLOR);

        const char *indicatorLabel = "Sentinel Drive ";

        RefSerialTransmitter::configCharacterMsg(
            BooleanHudIndicators::BOOLEAN_HUD_INDICATOR_LABEL_CHAR_SIZE,
            BooleanHudIndicators::BOOLEAN_HUD_INDICATOR_LABEL_CHAR_LINE_WIDTH,
            BooleanHudIndicators::BOOLEAN_HUD_INDICATOR_LIST_CENTER_X -
                strlen(indicatorLabel) *
                    BooleanHudIndicators::BOOLEAN_HUD_INDICATOR_LABEL_CHAR_SIZE -
                BooleanHudIndicators::BOOLEAN_HUD_INDICATOR_OUTLINE_RADIUS -
                BooleanHudIndicators::BOOLEAN_HUD_INDICATOR_OUTLINE_WIDTH / 2,
            SENTINEL_DRIVE_STATUS_Y +
                BooleanHudIndicators::BOOLEAN_HUD_INDICATOR_LABEL_CHAR_SIZE / 2,
            indicatorLabel,
            &this->indicatorText[i]);

        // configure frame IDs for the messages that need to be sent
        auto teamRobotId = drivers.refSerial.getRobotIdBasedOnCurrentRobotTeam(
            ROBOT_HUD_TO_UPDATE_TO_ROBOT_ID_MAP[i]);

        uint16_t robotClientId = getRobotClientID(teamRobotId);

        this->configFrameIdForSpecificRobot(&this->indicatorCircle[i], robotClientId);
        this->configFrameIdForSpecificRobot(&this->indicatorCircleOutline[i], robotClientId);
        this->configFrameIdForSpecificRobot(&this->indicatorText[i], robotClientId);
    }
}

bool SentinelDriveStatusHudIndicator::getChassisMovementStatus() const
{
    if (!this->drivers.commandScheduler.isCommandScheduled(&this->command))
    {
        return false;
    }

    return this->command.getMovementStatus();
}

void SentinelDriveStatusHudIndicator::configFrameIdForSpecificRobot(
    Tx::Graphic1Message *message,
    uint16_t clientRobotId)
{
    RefSerialTransmitter::configFrameHeader(
        &message->frameHeader,
        sizeof(message->graphicData) + sizeof(message->interactiveHeader));

    message->cmdId = RefSerial::REF_MESSAGE_TYPE_CUSTOM_DATA;

    RefSerialTransmitter::configInteractiveHeader(
        &message->interactiveHeader,
        0x101,
        drivers.refSerial.getRobotData().robotId,
        clientRobotId);

    message->crc16 = tap::algorithms::calculateCRC16(
        reinterpret_cast<uint8_t *>(message),
        sizeof(*message) - sizeof(message->crc16));
}

void SentinelDriveStatusHudIndicator::configFrameIdForSpecificRobot(
    Tx::GraphicCharacterMessage *message,
    uint16_t clientRobotId)
{
    RefSerialTransmitter::configFrameHeader(
        &message->frameHeader,
        sizeof(message->graphicData) + sizeof(message->interactiveHeader) +
            MODM_ARRAY_SIZE(message->msg));

    message->cmdId = RefSerial::REF_MESSAGE_TYPE_CUSTOM_DATA;

    RefSerialTransmitter::configInteractiveHeader(
        &message->interactiveHeader,
        0x110,
        drivers.refSerial.getRobotData().robotId,
        clientRobotId);

    message->crc16 = tap::algorithms::calculateCRC16(
        reinterpret_cast<uint8_t *>(message),
        sizeof(*message) - sizeof(message->crc16));
}

}  // namespace aruwsrc::control::client_display
