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

#include "vision_hud_indicators.hpp"

#include "aruwsrc/drivers.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
VisionHudIndicators::VisionHudIndicators(aruwsrc::Drivers *drivers) : drivers(drivers) {}

modm::ResumableResult<bool> VisionHudIndicators::sendInitialGraphics()
{
    RF_BEGIN(0);
    // Don't need to send anything since we add/delete the graphic when updating
    RF_END();
}

modm::ResumableResult<bool> VisionHudIndicators::update()
{
    RF_BEGIN(1);

    // update vision target status
    RF_CALL(updateVisionTargetStatus());

    RF_END();
}

modm::ResumableResult<bool> VisionHudIndicators::updateVisionTargetStatus()
{
    RF_BEGIN(2);

    currVisionTargetStatus = drivers->visionCoprocessor.isCvOnline() &&
                             drivers->visionCoprocessor.getSomeTurretHasTarget();

    if ((updateVisionTargetFoundTimeout.isExpired() ||
         updateVisionTargetFoundTimeout.isStopped()) &&
        prevVisionTargetStatus != currVisionTargetStatus)
    {
        visionTargetFoundGraphics.graphicData[0].operation =
            currVisionTargetStatus ? Tx::ADD_GRAPHIC : Tx::ADD_GRAPHIC_DELETE;
        visionTargetFoundGraphics.graphicData[1].operation =
            currVisionTargetStatus ? Tx::ADD_GRAPHIC : Tx::ADD_GRAPHIC_DELETE;

        drivers->refSerial.sendGraphic(&visionTargetFoundGraphics);

        DELAY_REF_GRAPHIC(&visionTargetFoundGraphics);

        updateVisionTargetFoundTimeout.restart(VISION_TARGET_FOUND_MAX_REFRESH_RATE);
        prevVisionTargetStatus = currVisionTargetStatus;
    }

    RF_END();
}

void VisionHudIndicators::initialize()
{
    prevVisionTargetStatus = false;

    static constexpr int CENTER_X_OFFSET =
        SCREEN_WIDTH / 2 + ReticleIndicator::RETICLE_CENTER_X_OFFSET;

    initializeVisionHudIndicator(
        &visionTargetFoundGraphics.graphicData[0],
        CENTER_X_OFFSET - VISION_TARGET_FOUND_X_DISTANCE_FROM_CENTER);
    initializeVisionHudIndicator(
        &visionTargetFoundGraphics.graphicData[1],
        CENTER_X_OFFSET + VISION_TARGET_FOUND_X_DISTANCE_FROM_CENTER);
}

void VisionHudIndicators::initializeVisionHudIndicator(
    Tx::GraphicData *graphicData,
    int xBoxLocation)
{
    uint8_t hudIndicatorName[3] = {};

    getUnusedGraphicName(hudIndicatorName);

    RefSerial::configGraphicGenerics(
        graphicData,
        hudIndicatorName,
        Tx::ADD_GRAPHIC,
        DEFAULT_GRAPHIC_LAYER,
        VISION_TARGET_FOUND_COLOR);

    RefSerial::configLine(
        VISION_TARGET_FOUND_SQUARE_WIDTH,
        xBoxLocation,
        VISION_TARGET_FOUND_Y_LOCATION - VISION_TARGET_FOUND_SQUARE_WIDTH / 2,
        xBoxLocation,
        VISION_TARGET_FOUND_Y_LOCATION + VISION_TARGET_FOUND_SQUARE_WIDTH / 2,
        graphicData);
}

}  // namespace aruwsrc::control::client_display
