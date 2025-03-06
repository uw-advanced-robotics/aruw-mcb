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

#ifndef UPDATE_SPEED_INDICATOR_HPP_
#define UPDATE_SPEED_INDICATOR_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display
{
using namespace tap::communication::serial;

class UpdateSpeedIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Write numbers as fast as possible to the screen.
     *
     */
    UpdateSpeedIndicator(tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    void initialize() override final
    {
        uint8_t indicatorName[3];

        getUnusedGraphicName(indicatorName);
        RefSerialTransmitter::configGraphicGenerics(
            &numberGraphic.graphicData,
            indicatorName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            Tx::GraphicColor::GREEN);

        RefSerialTransmitter::configInteger(
            30,
            5,
            SCREEN_WIDTH / 2,
            SCREEN_HEIGHT / 2,
            0,
            &numberGraphic.graphicData);
    }

    modm::ResumableResult<void> sendInitialGraphics() override final
    {
        RF_BEGIN(0);

        RF_CALL(refSerialTransmitter.sendGraphic(&numberGraphic));

        RF_END();
    }

    modm::ResumableResult<void> update()
    {
        numberGraphic.graphicData.operation = Tx::GRAPHIC_MODIFY;
        uint32_t time = tap::arch::clock::getTimeMilliseconds();
        RefSerialTransmitter::configInteger(
            30,
            5,
            SCREEN_WIDTH / 2,
            SCREEN_HEIGHT / 2,
            time,
            &numberGraphic.graphicData);
        RF_BEGIN(0);
        RF_CALL(refSerialTransmitter.sendGraphic(&numberGraphic));
        RF_END();
    }

private:
    Tx::Graphic1Message numberGraphic;
};

}  // namespace aruwsrc::control::client_display

#endif  // UPDATE_SPEED_INDICATOR_HPP_
