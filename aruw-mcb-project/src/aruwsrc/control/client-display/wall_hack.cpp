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

#include "wall_hack.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
WallHack::WallHack(
    aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
    RefSerialTransmitter &refSerialTransmitter,
    TransformerInterface *transformer)
    : HudIndicator(refSerialTransmitter),
      visionCoprocessor(visionCoprocessor),
      transformer(transformer),
      enemyPosition(0, 0, 0)
{
    // This is here otherwise the compiler compains that this is unused
    convertWorldFrameToScreenFrame(enemyPosition, transformer->getWorldToTurret(0));
}

float Y_OFFSET = 0.0f;
float Z_OFFSET = -1.0f;
modm::ResumableResult<bool> WallHack::update()
{
    VTM_OFFSET_FRAME = Position(0, Y_OFFSET, Z_OFFSET);

    auto aimData = visionCoprocessor.getLastAimData(0);
    bool visionHasTarget = visionCoprocessor.getSomeTurretHasTarget();

    // Get position
    enemyPosition = Position(aimData.pva.xPos, aimData.pva.yPos, aimData.pva.zPos);

    enemyPosScreenFrame =
        convertWorldFrameToScreenFrame(enemyPosition, transformer->getWorldToTurret(0));

    bottomLeftScreenFrame = convertWorldFrameToScreenFrame(
        enemyPosition - plateCornerOffset,
        transformer->getWorldToTurret(0));
    topRightScreenFrame = convertWorldFrameToScreenFrame(
        enemyPosition - (plateCornerOffset * -1),
        transformer->getWorldToTurret(0));

    RF_BEGIN(0);

    if (!enemyPosScreenFrame.inFrame)
    {
        // RF_RETURN(false);
    }

    visionTargetGraphic.graphicData.operation =
        visionTargetGraphic.graphicData.operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD
                                                                        : Tx::GRAPHIC_MODIFY;

    RefSerialTransmitter::configRectangle(
        WALL_HACK_THICKNESS,
        bottomLeftScreenFrame.screenX,
        bottomLeftScreenFrame.screenY,
        topRightScreenFrame.screenX,
        topRightScreenFrame.screenY,
        &visionTargetGraphic.graphicData);

    visionTargetGraphic.graphicData.color =
        static_cast<uint32_t>(visionHasTarget ? Tx::GraphicColor::GREEN : Tx::GraphicColor::ORANGE);

    RF_CALL(refSerialTransmitter.sendGraphic(&visionTargetGraphic));

    RF_END();
}

modm::ResumableResult<bool> WallHack::sendInitialGraphics()
{
    RF_BEGIN(1);
    RF_END();
}

void WallHack::initialize()
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
