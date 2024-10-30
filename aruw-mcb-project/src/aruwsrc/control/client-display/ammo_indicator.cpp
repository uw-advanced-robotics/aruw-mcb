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

bool initialized = false;
void AmmoIndicator::initialize()
{
    initialized = true;
    uint8_t bulletsRemainingName[3];

    getUnusedGraphicName(bulletsRemainingName);
    RefSerialTransmitter::configGraphicGenerics(
        &bulletsRemainingGraphics.graphicData,
        bulletsRemainingName,
        Tx::GRAPHIC_DELETE,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::YELLOW);

    RefSerialTransmitter::configCharacterMsg(
        TEXT_SIZE,
        TEXT_WIDTH,
        TEXT_X,
        TEXT_Y,
        "temp",
        &bulletsRemainingGraphics);
}

int initialGraphicsSent = 0;
modm::ResumableResult<bool> AmmoIndicator::sendInitialGraphics()
{
    RF_BEGIN(0)

    RF_CALL(refSerialTransmitter.sendGraphic(&bulletsRemainingGraphics));
    initialGraphicsSent++;

    RF_END();
}

int newBulletsPurchased = 0;
int updateCount = 0;
int updateOuter = 0;
modm::ResumableResult<bool> AmmoIndicator::update()
{
    updateOuter++;

    int ammoCount = refSerial.getRobotData().turret.bulletsRemaining42;
    if (ammoCount == lastBullets)
    {
        return false;
    }

    newBulletsPurchased++;


    lastBullets = ammoCount;

    const char *bulletsRemainingText = "AMMO: ";
    char bulletsRemainingTextBuffer[29];
    snprintf(bulletsRemainingTextBuffer, 29, "%s%d", bulletsRemainingText, ammoCount);

    RF_BEGIN(1);

    updateCount++;

        // Set the graphic state and update data
    bulletsRemainingGraphics.graphicData.operation =
        bulletsRemainingGraphics.graphicData.operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD
                                                                             : Tx::GRAPHIC_MODIFY;

    strncpy(bulletsRemainingGraphics.msg, bulletsRemainingTextBuffer, strlen(bulletsRemainingTextBuffer) + 1);

    bulletsRemainingGraphics.graphicData.endAngle = strlen(bulletsRemainingTextBuffer) + 1;  // Sets the length of the string

    RF_CALL(refSerialTransmitter.sendGraphic(&bulletsRemainingGraphics));

    RF_END();
}

}  // namespace aruwsrc::control::client_display
