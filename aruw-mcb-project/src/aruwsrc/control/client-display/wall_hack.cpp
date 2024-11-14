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
      enemyPositionWorldFrame(0, 0, 0),
      enemyPositionTurretFrame(0, 0, 0),
      enemyPositionVTMFrame(0, 0, 0),
      enemyPositionCameraAxes(0, 0, 0),
      enemyPositionScreenFrame(0, 0, 0)
{
    setProjectionMatrix();
}

modm::ResumableResult<bool> WallHack::update()
{
    if (redoMatrix)
    {
        setProjectionMatrix();
        redoMatrix = false;
    }

    VTM_OFFSET_FRAME = Position(0, 0, Z_OFFSET);

    auto aimData = visionCoprocessor.getLastAimData(0);

    // Get pos
    enemyPositionWorldFrame = Position(aimData.pva.xPos, aimData.pva.yPos, aimData.pva.zPos);
    copyToVector3(worldFrame, enemyPositionWorldFrame);

    // Convert to turret frame
    enemyPositionTurretFrame = transformer->getWorldToTurret(0).apply(enemyPositionWorldFrame);
    copyToVector3(turretFrame, enemyPositionTurretFrame);
    enemyPositionVTMFrame = enemyPositionTurretFrame + VTM_OFFSET_FRAME;

    // Define plate points
    CMSISMat<3, 1> enemyPositionVector = enemyPositionVTMFrame.coordinates();

    CMSISMat<3, 1> topRight = enemyPositionVector;
    topRight.data[1] += PLATE_SIZE_M / 2;
    topRight.data[2] += PLATE_SIZE_M / 2;

    CMSISMat<3, 1> bottomLeft = enemyPositionVector;
    bottomLeft.data[1] -= PLATE_SIZE_M / 2;
    bottomLeft.data[2] -= PLATE_SIZE_M / 2;

    enemyPositionScreenFrame = convertVectorByProjectionMatrix(enemyPositionVector);
    copyToVector3(screenFrame, enemyPositionScreenFrame);

    modm::Vector3f topRightScreenFrame = convertVectorByProjectionMatrix(topRight);
    modm::Vector3f bottomLeftScreenFrame = convertVectorByProjectionMatrix(bottomLeft);

    computedScreenX = std::clamp(
        (int)((enemyPositionScreenFrame.x + 1) * 0.5f * SCREEN_WIDTH),
        0,
        (SCREEN_WIDTH - 1));
    computedScreenY = std::clamp(
        (int)((enemyPositionScreenFrame.y + 1) * 0.5f * SCREEN_HEIGHT),
        0,
        SCREEN_HEIGHT - 1);

    uint32_t bottomLeftX = std::clamp(
        (int)((bottomLeftScreenFrame.x + 1) * 0.5f * SCREEN_WIDTH) + PIXEL_OFFSET_X,
        0,
        SCREEN_WIDTH - 1);
    uint32_t bottomLeftY = std::clamp(
        (int)((bottomLeftScreenFrame.y + 1) * 0.5f * SCREEN_HEIGHT),
        0,
        SCREEN_HEIGHT - 1);
    
    uint32_t topRightX = std::clamp(
        (int)((topRightScreenFrame.x + 1) * 0.5f * SCREEN_WIDTH) + PIXEL_OFFSET_X,
        0,
        SCREEN_WIDTH - 1);
    uint32_t topRightY = std::clamp(
        (int)((topRightScreenFrame.y + 1) * 0.5f * SCREEN_HEIGHT),
        0,
        SCREEN_HEIGHT - 1);

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
        bottomLeftX,
        bottomLeftY,
        topRightX,
        topRightY,
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
    float horizontalScale = 1.0f / tanf(horizontalFOV * 0.5 * M_PI / 180);
    float verticalScale = 1.0f / tanf(verticalFOV * 0.5 * M_PI / 180);

    projectionMatrix.data[0] = horizontalScale;
    projectionMatrix.data[4 * 1 + 1] = verticalScale;

    projectionMatrix.data[4 * 2 + 2] = -far / (far - near);
    projectionMatrix.data[4 * 2 + 3] = -1.0f;

    projectionMatrix.data[4 * 3 + 2] = -far * near / (far - near);
    projectionMatrix.data[4 * 3 + 3] = 0;
}

modm::Vector3f WallHack::convertVectorByProjectionMatrix(CMSISMat<3, 1> &vector)
{
    vector = swapAxesMatrix * vector;

    CMSISMat<4, 1> vec;
    vec.data[0] = vector.data[0];
    vec.data[1] = vector.data[1];
    vec.data[2] = vector.data[2];
    vec.data[3] = 1.0f;

    CMSISMat<4,4> temp = cameraDistortionMatrix * projectionMatrix.inverse();

    CMSISMat<4, 1> result = projectionMatrix * temp * vec;

    modm::Vector3f resultVec;
    resultVec.x = result.data[0] / result.data[3];
    resultVec.y = result.data[1] / result.data[3];
    resultVec.z = result.data[2] / result.data[3];

    return resultVec;
}

}  // namespace aruwsrc::control::client_display
