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

#include "vision_target_indicator.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
VisionTargetIndicator::VisionTargetIndicator(
    aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
    RefSerialTransmitter &refSerialTransmitter,
    const Transform &worldToCameraTransform)
    : HudIndicator(refSerialTransmitter),
      visionCoprocessor(visionCoprocessor),
      worldToCameraTransform(worldToCameraTransform),
      enemyPosition(0, 0, 0)
{
}

void VisionTargetIndicator::update()
{
    auto aimData = visionCoprocessor.getLastAimData(0);
    bool visionHasTarget = visionCoprocessor.getSomeTurretHasTarget();

    // Get position
    enemyPosition = Position(aimData.pva.xPos, aimData.pva.yPos, aimData.pva.zPos);

    enemyPositionScreenFrame = getEnemyPlatePosition(enemyPosition);

    uint32_t prevOperation = visionTargetGraphic.graphicData.operation;

    // If the target is not in frame, delete the graphic
    if (!enemyPositionScreenFrame.inFrame || !visionHasTarget)
    {
        visionTargetGraphic.graphicData.operation = Tx::GRAPHIC_DELETE;
    }
    else
    {
        visionTargetGraphic.graphicData.operation =
            prevOperation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD : Tx::GRAPHIC_MODIFY;
    }

    // If the graphic is already deleted, don't delete it again
    if (prevOperation == Tx::GRAPHIC_DELETE &&
        visionTargetGraphic.graphicData.operation == Tx::GRAPHIC_DELETE)
    {
    }

    RefSerialTransmitter::configRectangle(
        INDICATOR_LINE_THICKNESS,
        enemyPositionScreenFrame.bottomLeftX,
        enemyPositionScreenFrame.bottomLeftY,
        enemyPositionScreenFrame.topRightX,
        enemyPositionScreenFrame.topRightY,
        &visionTargetGraphic.graphicData);

    refSerialTransmitter.sendGraphic(&visionTargetGraphic);
}

void VisionTargetIndicator::initialize()
{
    uint8_t indicatorName[3];

    getUnusedGraphicName(indicatorName);
    RefSerialTransmitter::configGraphicGenerics(
        &visionTargetGraphic.graphicData,
        indicatorName,
        Tx::GRAPHIC_DELETE,
        DEFAULT_GRAPHIC_LAYER,
        INDICATOR_COLOR);
}

VisionTargetIndicator::ProjectedPlateResult VisionTargetIndicator::getEnemyPlatePosition(
    Position &enemyPositionWorldFrame)
{
    VisionTargetIndicator::ProjectedPlateResult output;

    Position cameraFrame = worldToCameraTransform.apply(enemyPositionWorldFrame);

    ProjectedResult screenFrame = convertCameraFrameToScreenFrame(cameraFrame);

    output.inFrame = screenFrame.inFrame;

    ProjectedResult topRight = convertCameraFrameToScreenFrame(cameraFrame + plateCornerOffset);

    ProjectedResult bottomLeft =
        convertCameraFrameToScreenFrame(cameraFrame + (plateCornerOffset * -1));

    output.bottomLeftX = bottomLeft.screenX;
    output.bottomLeftY = bottomLeft.screenY;
    output.topRightX = topRight.screenX;
    output.topRightY = topRight.screenY;

    return output;
}

}  // namespace aruwsrc::control::client_display
