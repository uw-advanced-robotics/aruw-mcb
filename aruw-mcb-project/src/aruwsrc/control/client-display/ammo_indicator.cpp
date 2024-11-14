/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "ammo_indicator.hpp"

#include "tap/drivers.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
AmmoIndicator::AmmoIndicator(RefSerialTransmitter &refSerialTransmitter, const RefSerial &refSerial)
    : HudIndicator(refSerialTransmitter),
      refSerial(refSerial)
{
}

void AmmoIndicator::initialize()
{
    uint8_t bulletsRemainingName[3];

    getUnusedGraphicName(bulletsRemainingName);
    RefSerialTransmitter::configGraphicGenerics(
        &bulletsRemainingGraphics.graphicData,
        bulletsRemainingName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::ORANGE);

    RefSerialTransmitter::configCharacterMsg(
        TEXT_SIZE,
        TEXT_WIDTH,
        TEXT_X,
        TEXT_Y,
        "",
        &bulletsRemainingGraphics);
}

modm::ResumableResult<bool> AmmoIndicator::sendInitialGraphics()
{
    RF_BEGIN(0)

    RF_CALL(refSerialTransmitter.sendGraphic(&bulletsRemainingGraphics));
    updateTimer.restart(500);

    RF_END();
}

modm::ResumableResult<bool> AmmoIndicator::update()
{
    int prevBulletCount = bulletCount;
    uint32_t prevOp = bulletsRemainingGraphics.graphicData.operation;

    RF_BEGIN(1);

    // If we aren't look to update the graphic, return early
    // This ensure we haven't updated bullet count, so if it has changed we will update the graphic
    if (!updateTimer.execute())
    {
        RF_RETURN(false);
    }

    // Access the correct field depending on the robot type
    if (refSerial.getRobotData().robotId == RefSerialData::RobotId::BLUE_HERO ||
        refSerial.getRobotData().robotId == RefSerialData::RobotId::RED_HERO)
    {
        bulletCount = refSerial.getRobotData().turret.bulletsRemaining42;
    }
    else
    {
        bulletCount = refSerial.getRobotData().turret.bulletsRemaining17;
    }

    // Appends the current bullet count to the "AMMO: " text
    snprintf(
        bulletsRemainingTextBuffer,
        TEXT_BUFFER_SIZE,
        "%s%hd",
        bulletsRemainingText,
        bulletCount);

    // If we previously deleted the graphic, we need to add it back
    // If not, we are trying to update the ammo count
    bulletsRemainingGraphics.graphicData.operation =
        bulletsRemainingGraphics.graphicData.operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD
                                                                             : Tx::GRAPHIC_MODIFY;

    // Copy over the text into the graphics message
    strncpy(bulletsRemainingGraphics.msg, bulletsRemainingTextBuffer, TEXT_BUFFER_SIZE);

    // Updates the length of the string, needed as on initialization it is 0 length string
    bulletsRemainingGraphics.graphicData.endAngle = TEXT_BUFFER_SIZE;

    // If the bullet count has changed, or the operation has changed, send the graphic
    if (prevBulletCount != bulletCount || prevOp != bulletsRemainingGraphics.graphicData.operation)
    {
        RF_CALL(refSerialTransmitter.sendGraphic(&bulletsRemainingGraphics));
    }

    RF_END();
}

}  // namespace aruwsrc::control::client_display
