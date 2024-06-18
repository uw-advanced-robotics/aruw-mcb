/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "ref_system_indicators.hpp"

#include <cstdlib>

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/drivers.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
RefSystemIndicators::RefSystemIndicators(
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
    const tap::communication::serial::RefSerial &refSerial)
    : HudIndicator(refSerialTransmitter),
      refSerial(refSerial)
{
}

modm::ResumableResult<bool> RefSystemIndicators::sendInitialGraphics()
{
    RF_BEGIN(0)

#if defined(TARGET_HERO_PERSEUS)
    // remove initial graphics
    RF_CALL(refSerialTransmitter.sendGraphic(&heroAmmoCountTextGraphic));
#endif

    RF_END();
}

modm::ResumableResult<bool> RefSystemIndicators::update()
{
#if defined(TARGET_HERO_PERSEUS)
    int heroAmmoCount = refSerial.getRobotData().turret.bulletsRemaining42;

    if (heroAmmoCount == lastHeroAmmoCount)
    {
        return;
    }

    lastHeroAmmoCount = heroAmmoCount;

    const char *heroAmmoCountStr =
        std::to_string(heroAmmoCount).append(std::string(heroAmmoCount < 10 ? " " : "")).c_str();

    RF_BEGIN(1);

    heroAmmoCountTextGraphic.graphicData.operation =
        heroAmmoCountTextGraphic.graphicData.operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD
                                                                             : Tx::GRAPHIC_MODIFY;

    strncpy(heroAmmoCountTextGraphic.msg, heroAmmoCountStr, 3);

    // Update the text
    heroAmmoCountTextGraphic.graphicData.endAngle = 3;  // Sets the length of the string

    RF_CALL(refSerialTransmitter.sendGraphic(&heroAmmoCountTextGraphic));
    RF_END();
#endif
}

void RefSystemIndicators::initialize()
{
#if defined(TARGET_HERO_PERSEUS)
    uint8_t heroAmmoCountName[3];

    getUnusedGraphicName(heroAmmoCountName);
    RefSerialTransmitter::configGraphicGenerics(
        &heroAmmoCountTextGraphic.graphicData,
        heroAmmoCountName,
        Tx::GRAPHIC_DELETE,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::YELLOW);

    RefSerialTransmitter::configCharacterMsg(
        AMMO_COUNT_CHAR_SIZE,
        2,
        AMMO_COUNT_CENTER_X,
        AMMO_COUNT_CENTER_X,
        "",
        &heroAmmoCountTextGraphic);
#endif
}
}  // namespace aruwsrc::control::client_display
