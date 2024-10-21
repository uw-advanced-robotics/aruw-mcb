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

#include "hero_assist_indicator.hpp"

#include "tap/drivers.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
HeroAssistIndicator::HeroAssistIndicator(
    RefSerialTransmitter &refSerialTransmitter,
    const RefSerial &refSerial)
    : HudIndicator(refSerialTransmitter),
      refSerial(refSerial)
{
}

modm::ResumableResult<bool> HeroAssistIndicator::sendInitialGraphics()
{
    RF_BEGIN(0)

    // remove initial graphics
    RF_CALL(refSerialTransmitter.sendGraphic(&bulletsRemainingGraphics));
    RF_CALL(refSerialTransmitter.sendGraphic(&num42NeededGraphics));

    RF_END();
}

void HeroAssistIndicator::initialize()
{
    uint8_t bulletsRemainingName[3];
    uint8_t num42NeededName[3];

    getUnusedGraphicName(bulletsRemainingName);
    RefSerialTransmitter::configGraphicGenerics(
        &bulletsRemainingGraphics.graphicData,
        bulletsRemainingName,
        Tx::GRAPHIC_DELETE,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::YELLOW);

    RefSerialTransmitter::configCharacterMsg(
        TEXT_WIDTH * 10,
        TEXT_WIDTH,
        TEXT_X,
        BULLETS_Y,
        "",
        &bulletsRemainingGraphics);

    getUnusedGraphicName(num42NeededName);
    RefSerialTransmitter::configGraphicGenerics(
        &num42NeededGraphics.graphicData,
        num42NeededName,
        Tx::GRAPHIC_DELETE,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::YELLOW);

    RefSerialTransmitter::configCharacterMsg(
        TEXT_WIDTH * 10,
        TEXT_WIDTH,
        TEXT_X,
        NUM_42_NEEDED_Y,
        "",
        &num42NeededGraphics);
}

modm::ResumableResult<bool> HeroAssistIndicator::update()
{
    RF_BEGIN(1);

    RF_END();
}

modm::ResumableResult<bool> HeroAssistIndicator::updateBulletsRemaining()
{

    int ammoCount = refSerial.getRobotData().turret.bulletsRemaining42;
    if (ammoCount == lastBullets)
    {
        return false;
    }

    lastBullets = ammoCount;

    const char *bulletsRemainingText = "AMMO: ";
    char bulletsRemainingTextBuffer[TEXT_WIDTH];
    snprintf(bulletsRemainingTextBuffer, TEXT_WIDTH, "%s%d", bulletsRemainingText, ammoCount);

    // Set the graphic state and update data
    bulletsRemainingGraphics.graphicData.operation =
        bulletsRemainingGraphics.graphicData.operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD
                                                                             : Tx::GRAPHIC_MODIFY;
    strncpy(bulletsRemainingGraphics.msg, bulletsRemainingTextBuffer, TEXT_WIDTH);

    RF_BEGIN(2);

    RF_CALL(refSerialTransmitter.sendGraphic(&bulletsRemainingGraphics));

    RF_END();
}

modm::ResumableResult<bool> HeroAssistIndicator::updateNum42Needed()
{

    bool isBlue = refSerial.isBlueTeam(refSerial.getRobotData().robotId);

    int num42Needed = 99;
    if (isBlue)
    {
        num42Needed =
            refSerial.getRobotData().allRobotHp.red.base / 200 + 1;  // Always have 1 extra
    }
    else
    {
        num42Needed =
            refSerial.getRobotData().allRobotHp.blue.base / 200 + 1;  // Always have 1 extra
    }

    if (num42Needed == lastNum42Needed)
    {
        return false;
    }

    lastNum42Needed = num42Needed;

    const char *num42NeededText = "NEED: ";
    char num42NeededTextBuffer[TEXT_WIDTH];
    snprintf(num42NeededTextBuffer, TEXT_WIDTH, "%s%d", num42NeededText, num42Needed);

    // Set the graphic state and update data
    num42NeededGraphics.graphicData.operation =
        num42NeededGraphics.graphicData.operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD
                                                                        : Tx::GRAPHIC_MODIFY;
    strncpy(num42NeededGraphics.msg, num42NeededTextBuffer, TEXT_WIDTH);

    RF_BEGIN(3);

    RF_CALL(refSerialTransmitter.sendGraphic(&num42NeededGraphics));

    RF_END();
}

}  // namespace aruwsrc::control::client_display
