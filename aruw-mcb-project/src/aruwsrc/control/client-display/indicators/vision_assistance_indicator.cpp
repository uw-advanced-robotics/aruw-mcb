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

#include "vision_assistance_indicator.hpp"

namespace aruwsrc::control::client_display
{
VisionAssistanceIndicator::VisionAssistanceIndicator(
    aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
    tap::communication::serial::RefSerial &refSerial,
    const Transform &worldToCameraTransform,
    InterRobotTransmitter &interRobotTransmitter)
    : HudIndicator(refSerialTransmitter),
      visionCoprocessor(visionCoprocessor),
      refSerial(refSerial),
      worldToCameraTransform(worldToCameraTransform),
      interRobotTransmitter(interRobotTransmitter)
{
}

void VisionAssistanceIndicator::initialize()
{
    configureGraphic(GraphicIndex::TARGET, Tx::GraphicColor::GREEN);

    configureGraphic(GraphicIndex::HERO_TRACER, Tx::GraphicColor::ORANGE);
    configureGraphic(GraphicIndex::STANDARD_TRACER, Tx::GraphicColor::ORANGE);
    configureGraphic(GraphicIndex::SENTRY_TRACER, Tx::GraphicColor::ORANGE);

    // Don't set these up till we have icon data
    configureGraphic(GraphicIndex::HERO_HP, Tx::GraphicColor::PURPLISH_RED);
    configureGraphic(GraphicIndex::STANDARD_HP, Tx::GraphicColor::PURPLISH_RED);
    configureGraphic(GraphicIndex::SENTRY_HP, Tx::GraphicColor::PURPLISH_RED);
}

modm::ResumableResult<void> VisionAssistanceIndicator::sendInitialGraphics()
{
    RF_BEGIN(0);

    // Send the graphics
    RF_CALL(refSerialTransmitter.sendGraphic(&graphic));

    RF_END();
}

modm::ResumableResult<void> VisionAssistanceIndicator::update()
{
    // Variable definitions because protothread can't
    bool visionHasTarget;
    InterRobotTransmitter::EnemyRobotState robotOrbits;

    RF_BEGIN(1);
    visionHasTarget = visionCoprocessor.getSomeTurretHasTarget();

    if (!visionHasTarget)
    {
        deleteGraphic(GraphicIndex::TARGET);
    }
    else
    {
        // Draw the target box
        drawPlateTargetBox();
    }

    robotOrbits = interRobotTransmitter.getStateEstimate();

    for (int i = 0; i < aruwsrc::serial::VisionCoprocessor::MAX_NUM_ROBOT_ORBITS; i++)
    {
        bool valid = (tap::arch::clock::getTimeMilliseconds() - robotOrbits.robot[i].timestamp) <
                     TIME_CUTOFF_MS;
        GraphicIndex tracerIndex = static_cast<GraphicIndex>(i * 2 + 1);
        GraphicIndex healthBarIndex = static_cast<GraphicIndex>(i * 2 + 2);
        if (valid)
        {
            Position orbit =
                Position(robotOrbits.robot[i].x, robotOrbits.robot[i].y, robotOrbits.robot[i].z);

            // Draw the tracer line to the orbit
            drawTracerLineToOrbit(orbit, tracerIndex);

            // Don't draw health bars for not, with no icon data it will just show 0
            drawHealthBarToOrbit(orbit, healthBarIndex, robotOrbits.robot[i].robotType);
        }
        else
        {
            deleteGraphic(tracerIndex);
            deleteGraphic(healthBarIndex);
        }
    }

    // Send the graphics
    RF_CALL(refSerialTransmitter.sendGraphic(&graphic));
    RF_END();
}

void VisionAssistanceIndicator::drawTracerLineToOrbit(Position orbit, GraphicIndex index)
{
    Position cameraFrameOrbit = worldToCameraTransform.apply(orbit);
    ProjectedResult screenFrameOrbit =
        convertCameraFrameToScreenFrame(cameraFrameOrbit + TRACER_LINE_OFFSET);

    auto graphicToModify = &graphic.graphicData[static_cast<uint8_t>(index)];
    graphicToModify->operation =
        graphicToModify->operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD : Tx::GRAPHIC_MODIFY;

    if (screenFrameOrbit.inFrame)
    {
        // Make it ORANGE
        graphicToModify->color = static_cast<uint8_t>(Tx::GraphicColor::ORANGE);

        RefSerialTransmitter::configLine(
            1,
            TRACER_LINE_ORIGIN.x,
            TRACER_LINE_ORIGIN.y,
            screenFrameOrbit.screenX,
            screenFrameOrbit.screenY,
            graphicToModify);
    }
    else
    {
        // If the orbit is not in frame, draw a line offscreen to where they should be.
        float angle = atan2(cameraFrameOrbit.y(), cameraFrameOrbit.x()) + M_PI_2;
        angle = modm::Angle::normalize(angle);

        // Calculate the end point of the line, 10,000 pixels away from the origin cuz we want it to
        // go offscreen
        uint16_t x = TRACER_LINE_ORIGIN.x + 200 * cos(angle);
        uint16_t y = TRACER_LINE_ORIGIN.y + 200 * sin(angle);

        x = tap::algorithms::limitVal(x, static_cast<uint16_t>(0), SCREEN_WIDTH);
        y = tap::algorithms::limitVal(y, static_cast<uint16_t>(0), SCREEN_HEIGHT);

        // Make it PORPLE
        graphicToModify->color = static_cast<uint8_t>(Tx::GraphicColor::PURPLISH_RED);

        RefSerialTransmitter::configLine(
            1,
            TRACER_LINE_ORIGIN.x,
            TRACER_LINE_ORIGIN.y,
            x,
            y,
            graphicToModify);
    }
};

void VisionAssistanceIndicator::drawHealthBarToOrbit(Position orbit, GraphicIndex index, int ID)
{
    Position cameraFrameOrbit = worldToCameraTransform.apply(orbit);
    ProjectedResult screenFrameOrbit =
        convertCameraFrameToScreenFrame(cameraFrameOrbit + HEALTH_BAR_OFFSET);

    auto graphicToModify = &graphic.graphicData[static_cast<uint8_t>(index)];
    graphicToModify->operation =
        graphicToModify->operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD : Tx::GRAPHIC_MODIFY;

    // Show numerically the health of the target
    int robotHP = 0;
    auto robotHPData = refSerial.getRobotData().allRobotHp;
    if (refSerial.isBlueTeam(refSerial.getRobotData().robotId))
    {
        if (ID == 1)
            robotHP = robotHPData.red.hero1;
        else if (ID == 3)
            robotHP = robotHPData.red.standard3;
        else if (ID == 4)
            robotHP = robotHPData.red.standard4;
        else if (ID == 7)
            robotHP = robotHPData.red.sentry7;
    }
    else
    {
        if (ID == 1)
            robotHP = robotHPData.blue.hero1;
        else if (ID == 3)
            robotHP = robotHPData.blue.standard3;
        else if (ID == 4)
            robotHP = robotHPData.blue.standard4;
        else if (ID == 7)
            robotHP = robotHPData.blue.sentry7;
    }

    if (screenFrameOrbit.inFrame)
    {
        RefSerialTransmitter::configInteger(
            20,
            3,
            screenFrameOrbit.screenX,
            screenFrameOrbit.screenY,
            robotHP,
            graphicToModify);
    }
    else
    {
        deleteGraphic(index);
    }
}

void VisionAssistanceIndicator::drawPlateTargetBox()
{
    auto aimData = visionCoprocessor.getLastAimData(0);

    // Get position
    Position enemyPlatePosition = Position(aimData.pva.xPos, aimData.pva.yPos, aimData.pva.zPos);

    Position cameraFrameOrbit = worldToCameraTransform.apply(enemyPlatePosition);

    ProjectedResult screenFrameTopRight =
        convertCameraFrameToScreenFrame(cameraFrameOrbit + PLATE_CORNER_OFFSET);
    ProjectedResult screenFrameBottomLeft =
        convertCameraFrameToScreenFrame(cameraFrameOrbit - PLATE_CORNER_OFFSET);

    auto graphicToModify = &graphic.graphicData[static_cast<uint8_t>(GraphicIndex::TARGET)];

    graphicToModify->operation =
        graphicToModify->operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD : Tx::GRAPHIC_MODIFY;

    if (!screenFrameTopRight.inFrame)
    {
        graphicToModify->operation = Tx::GRAPHIC_DELETE;
    }

    RefSerialTransmitter::configRectangle(
        3,
        screenFrameBottomLeft.screenX,
        screenFrameBottomLeft.screenY,
        screenFrameTopRight.screenX,
        screenFrameTopRight.screenY,
        graphicToModify);
}

void VisionAssistanceIndicator::configureGraphic(GraphicIndex index, Tx::GraphicColor color)
{
    uint8_t idx = static_cast<uint8_t>(index);
    uint8_t graphicName[3];
    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &graphic.graphicData[idx],
        graphicName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        color);
}

void VisionAssistanceIndicator::deleteGraphic(GraphicIndex index)
{
    uint8_t idx = static_cast<uint8_t>(index);
    graphic.graphicData[idx].operation = Tx::GRAPHIC_DELETE;
}

}  // namespace aruwsrc::control::client_display
