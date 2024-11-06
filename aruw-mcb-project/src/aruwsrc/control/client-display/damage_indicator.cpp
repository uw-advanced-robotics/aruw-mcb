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

#include "damage_indicator.hpp"

#include "tap/architecture/clock.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{

DamageIndicator::DamageIndicator(
    aruwsrc::algorithms::PlateHitTracker &plateHitTracker,
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter)
    : HudIndicator(refSerialTransmitter),
      plateHitTracker(plateHitTracker)
{
}

modm::ResumableResult<bool> DamageIndicator::update()
{
    RF_BEGIN(1);

    // Get current damage angle
    auto hitData = plateHitTracker.getLastHitData();

    bool hasNewHitData = hitData.timestamp

        RF_END();
}

modm::ResumableResult<bool> DamageIndicator::sendInitialGraphics()
{
    RF_BEGIN(0);
    // Don't need to send anything since we add/delete the graphic when updating
    RF_END();
}

void DamageIndicator::initialize()
{
    uint8_t indicatorName[3];

    getUnusedGraphicName(indicatorName);
    RefSerialTransmitter::configGraphicGenerics(
        &leftGraphic.graphicData,
        indicatorName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::RED_AND_BLUE);

    RefSerialTransmitter::configLine(
        DAMAGE_INDICATOR_THICKNESS,
        DAMAGE_INDICATOR_LEFT_X,
        DAMAGE_INDICATOR_LR_Y_BOTTOM,
        DAMAGE_INDICATOR_LEFT_X,
        DAMAGE_INDICATOR_LR_Y_TOP,
        &leftGraphic.graphicData);

    getUnusedGraphicName(indicatorName);
    RefSerialTransmitter::configGraphicGenerics(
        &rightGraphic.graphicData,
        indicatorName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::RED_AND_BLUE);

    RefSerialTransmitter::configLine(
        DAMAGE_INDICATOR_THICKNESS,
        DAMAGE_INDICATOR_RIGHT_X,
        DAMAGE_INDICATOR_LR_Y_BOTTOM,
        DAMAGE_INDICATOR_RIGHT_X,
        DAMAGE_INDICATOR_LR_Y_TOP,
        &rightGraphic.graphicData);

    getUnusedGraphicName(indicatorName);
    RefSerialTransmitter::configGraphicGenerics(
        &bottomGraphic.graphicData,
        indicatorName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::RED_AND_BLUE);

    RefSerialTransmitter::configLine(
        DAMAGE_INDICATOR_THICKNESS,
        DAMAGE_INDICATOR_BOTTOM_X,
        DAMAGE_INDICATOR_BOTTOM_Y_BOTTOM,
        DAMAGE_INDICATOR_BOTTOM_X,
        DAMAGE_INDICATOR_BOTTOM_Y_TOP,
        &bottomGraphic.graphicData);
}

}  // namespace aruwsrc::control::client_display
