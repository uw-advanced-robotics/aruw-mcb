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

int updates;
int validDatas;

modm::ResumableResult<bool> DamageIndicator::update()
{
    RF_BEGIN(1);

    updates++;
    peakAngleBin = plateHitTracker.getPeakAnglesRadians()[0];

    // hasValidAngle = peakAngleBin.magnitude > DAMAGE_THRESHOLD;
    hasValidAngle = true;


    // Figure out X, Y cordinates for the line, add 90 deg cuz 0 to the right
    degreeRadian = peakAngleBin.radians.getWrappedValue();
    degree = modm::toDegree(degreeRadian);


    float offsetDegreeRadian = degreeRadian + INDICATOR_OFFSET_RADIANS;
    x = cos(offsetDegreeRadian) * DISTANCE_FROM_CENTER;
    y = sin(offsetDegreeRadian) * DISTANCE_FROM_CENTER;

    RefSerialTransmitter::configLine(
        DAMAGE_INDICATOR_THICKNESS,
        X_POS + x,
        Y_POS + y,
        X_POS + x,
        Y_POS + LINE_LENGTH + y,
        &damageGraphic.graphicData);

    if (hasValidAngle){
        auto prevOperation = damageGraphic.graphicData.operation;
        damageGraphic.graphicData.operation = prevOperation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD : Tx::GRAPHIC_MODIFY;
        validDatas++;
    } else
    {
        damageGraphic.graphicData.operation = Tx::GRAPHIC_DELETE;
    }

    RF_CALL(refSerialTransmitter.sendGraphic(&damageGraphic));    

    RF_END();
}

modm::ResumableResult<bool> DamageIndicator::sendInitialGraphics()
{
    RF_BEGIN(0);

    // We do this so that the graphic gets drawn to begin with
    RF_CALL(refSerialTransmitter.sendGraphic(&damageGraphic));

    RF_END();
}

void DamageIndicator::initialize()
{
    uint8_t indicatorName[3];

    getUnusedGraphicName(indicatorName);
    RefSerialTransmitter::configGraphicGenerics(
        &damageGraphic.graphicData,
        indicatorName,
        Tx::GRAPHIC_DELETE,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::PURPLISH_RED);

    RefSerialTransmitter::configLine(
        DAMAGE_INDICATOR_THICKNESS,
        SCREEN_WIDTH / 2,
        SCREEN_HEIGHT + DISTANCE_FROM_CENTER,
        SCREEN_WIDTH / 2,
        SCREEN_HEIGHT + DISTANCE_FROM_CENTER + DAMAGE_INDICATOR_LENGTH,
        &damageGraphic.graphicData);
}

}  // namespace aruwsrc::control::client_display
