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
    TransformerInterface *transformer)
    : HudIndicator(refSerialTransmitter),
      visionCoprocessor(visionCoprocessor),
      transformer(transformer),
      enemyPosition(0, 0, 0)
{
}

float Y_OFFSET = VTM_OFFSET_FRAME.y();
float Z_OFFSET = VTM_OFFSET_FRAME.z();
modm::ResumableResult<bool> VisionTargetIndicator::update()
{
    // Here for debugging purposes. Should be removed in the future
    VTM_OFFSET_FRAME = Position(0, Y_OFFSET, Z_OFFSET);

    auto aimData = visionCoprocessor.getLastAimData(0);
    bool visionHasTarget = visionCoprocessor.getSomeTurretHasTarget();

    if (visionHasTarget)
    {
        targetTimeout.restart(TIMEOUT_MS);
    }

    // Get position
    enemyPosition = Position(aimData.pva.xPos, aimData.pva.yPos, aimData.pva.zPos);

    if (square)
    {
        enemyPositionScreenFrame = getEnemyPositionPlateSquare(enemyPosition);
    }
    else
    {
        enemyPositionScreenFrame = getEnemyPositionPlateCircle(enemyPosition);
    }

    uint32_t prevOperation = visionTargetGraphic.graphicData.operation;

    RF_BEGIN(0);

    // If the target is not in frame or the target has timed out, delete the graphic
    if (!enemyPositionScreenFrame.inFrame || targetTimeout.isExpired())
    {
        visionTargetGraphic.graphicData.operation = Tx::GRAPHIC_DELETE;
    }
    else
    {
        visionTargetGraphic.graphicData.operation =
            prevOperation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD : Tx::GRAPHIC_MODIFY;
    }

    visionTargetGraphic.graphicData.color = static_cast<uint32_t>(
        visionHasTarget ? INDICATOR_HAS_TARGET_COLOR : INDICATOR_NO_TARGET_COLOR);

    // If the graphic is already deleted, don't delete it again
    if (prevOperation == Tx::GRAPHIC_DELETE &&
        visionTargetGraphic.graphicData.operation == Tx::GRAPHIC_DELETE)
    {
        RF_RETURN(true);
    }

    RefSerialTransmitter::configRectangle(
        INDICATOR_LINE_THICKNESS,
        enemyPositionScreenFrame.bottomLeftX,
        enemyPositionScreenFrame.bottomLeftY,
        enemyPositionScreenFrame.topRightX,
        enemyPositionScreenFrame.topRightY,
        &visionTargetGraphic.graphicData);

    RF_CALL(refSerialTransmitter.sendGraphic(&visionTargetGraphic));

    RF_END();
}

modm::ResumableResult<bool> VisionTargetIndicator::sendInitialGraphics()
{
    RF_BEGIN(1);
    RF_END();
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
        Tx::GraphicColor::PURPLISH_RED);
}

}  // namespace aruwsrc::control::client_display
