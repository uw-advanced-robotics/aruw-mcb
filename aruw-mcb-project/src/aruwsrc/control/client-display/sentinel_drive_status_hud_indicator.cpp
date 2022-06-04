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

    for (i = 0; i < MODM_ARRAY_SIZE(this->indicatorText); i++)
    {
        RF_CALL(this->refSerialTransmitter.sendGraphic(&this->indicatorText[i], false));
        RF_CALL(this->refSerialTransmitter.sendGraphic(&this->indicatorCircleOutline[i], false));
        RF_CALL(this->refSerialTransmitter.sendGraphic(&this->indicatorCircle[i], false));

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
        for (i = 0; i < MODM_ARRAY_SIZE(this->indicatorCircle); i++)
        {
            this->indicatorCircle[i].graphicData.operation = Tx::GraphicOperation::GRAPHIC_ADD;
            this->configFrameIdForSpecificRobot(
                &this->indicatorCircle[i],
                this->indicatorCircle[i].interactiveHeader.receiverId);
        }

        sendInitialGraphics();
    }
    else if (this->drivers.commandScheduler.isCommandScheduled(&this->command))
    {
        if (this->getChassisMovementStatus() != this->prevChassisMovementStatus)
        {
            this->prevChassisMovementStatus = this->getChassisMovementStatus();

            for (i = 0; i < MODM_ARRAY_SIZE(this->indicatorCircle); i++)
            {
                this->indicatorCircle[i].graphicData.color = static_cast<uint8_t>(
                    this->prevChassisMovementStatus ? Tx::GraphicColor::GREEN
                                                    : Tx::GraphicColor::PURPLISH_RED);

                this->configFrameIdForSpecificRobot(
                    &this->indicatorCircle[i],
                    this->indicatorCircle[i].interactiveHeader.receiverId);

                RF_CALL(this->refSerialTransmitter.sendGraphic(&this->indicatorCircle[i], false));
            }
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
    for (size_t i = 0; i < MODM_ARRAY_SIZE(this->indicatorCircle); i++)
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
            Tx::GraphicColor::PURPLISH_RED);

        RefSerialTransmitter::configCircle(
            BooleanHudIndicators::BOOLEAN_HUD_INDICATOR_WIDTH,
            BooleanHudIndicators::BOOLEAN_HUD_INDICATOR_LIST_CENTER_X,
            660,
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
            660,
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
            660 + BooleanHudIndicators::BOOLEAN_HUD_INDICATOR_LABEL_CHAR_SIZE / 2,
            indicatorLabel,
            &this->indicatorText[i]);
    }

    uint16_t heroRobotId = 0;
    uint16_t soldierRobotId = 0;

    if (RefSerialData::isBlueTeam(drivers.refSerial.getRobotData().robotId))
    {
        heroRobotId = getRobotClientID(RobotId::BLUE_HERO);
        soldierRobotId = getRobotClientID(RobotId::BLUE_SOLDIER_1);
    }
    else
    {
        heroRobotId = getRobotClientID(RobotId::RED_HERO);
        soldierRobotId = getRobotClientID(RobotId::RED_SOLDIER_1);
    }

    this->configFrameIdForSpecificRobot(&this->indicatorCircle[0], heroRobotId);
    this->configFrameIdForSpecificRobot(&this->indicatorCircle[1], soldierRobotId);

    this->configFrameIdForSpecificRobot(&this->indicatorCircleOutline[0], heroRobotId);
    this->configFrameIdForSpecificRobot(&this->indicatorCircleOutline[1], soldierRobotId);

    this->configFrameIdForSpecificRobot(&this->indicatorText[0], heroRobotId);
    this->configFrameIdForSpecificRobot(&this->indicatorText[1], soldierRobotId);
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
