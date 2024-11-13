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
      transformer(transformer)
{
    setProjectionMatrix();
}

modm::ResumableResult<bool> WallHack::update()
{
    auto robotPos = transformer->getWorldToTurret(0);
    robotPositionVector.x = robotPos.getX();
    robotPositionVector.y = robotPos.getY();
    robotPositionVector.z = robotPos.getZ();

    auto aimData = visionCoprocessor.getLastAimData(0);

    Position enemyPosition = Position(aimData.pva.xPos, aimData.pva.yPos, aimData.pva.zPos);
    enemyPosition = transformer->getWorldToTurret(0).apply(enemyPosition);

    // Swap axes
    enemyPosition = swapAxesMatrix * enemyPosition.coordinates();
    enemyPositionVector.x = enemyPosition.x();
    enemyPositionVector.y = enemyPosition.y();
    enemyPositionVector.z = enemyPosition.z();

    enemyPositionTransformed = convertVectorByProjectionMatrix(enemyPositionVector);

    computedScreenX = std::min(
        (uint32_t)(SCREEN_WIDTH - 1),
        (uint32_t)((enemyPositionTransformed.x + 1) * 0.5 * SCREEN_WIDTH));
    computedScreenY = std::min(
        (uint32_t)(SCREEN_HEIGHT - 1),
        (uint32_t)((1 - enemyPositionTransformed.y) * 0.5 * SCREEN_HEIGHT));

    RF_BEGIN(0);

    // if (abs(enemyPositionTransformed.x) > 1 || abs(enemyPositionTransformed.y) > 1)
    // {
    //     RF_RETURN(false);
    // }

    visionTargetGraphic.graphicData.operation =
        visionTargetGraphic.graphicData.operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD
                                                                        : Tx::GRAPHIC_MODIFY;

    RefSerialTransmitter::configRectangle(
        WALL_HACK_THICKNESS,
        computedScreenX,
        computedScreenY,
        computedScreenX + SQUARE_SIZE,
        computedScreenY + SQUARE_SIZE,
        &visionTargetGraphic.graphicData);

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

void WallHack::setProjectionMatrix()
{
    float horizontalScale = 1 / tanf(horizontalFOV * 0.5 * M_PI / 180);
    float verticalScale = 1 / tanf(verticalFOV * 0.5 * M_PI / 180);

    projectionMatrix.data[0] = horizontalScale;
    projectionMatrix.data[4 * 1 + 1] = verticalScale;

    projectionMatrix.data[4 * 2 + 2] = -far / (far - near);
    projectionMatrix.data[4 * 2 + 3] = -1;

    projectionMatrix.data[4 * 3 + 2] = -far * near / (far - near);
    projectionMatrix.data[4 * 3 + 3] = 0;
}

modm::Vector3f WallHack::convertVectorByProjectionMatrix(modm::Vector3f &vector)
{
    CMSISMat<4, 1> vec;
    vec.data[0] = vector.x;
    vec.data[1] = vector.y;
    vec.data[2] = vector.z;
    vec.data[3] = 1;

    CMSISMat<4, 1> result = projectionMatrix * vec;

    modm::Vector3f resultVec;
    resultVec.x = result.data[0] / result.data[3];
    resultVec.y = result.data[1] / result.data[3];
    resultVec.z = result.data[2] / result.data[3];

    return resultVec;
}

}  // namespace aruwsrc::control::client_display
