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

#include "driver_assistance_indicator.hpp"

namespace aruwsrc::control::client_display
{
DriverAssistanceIndicator::DriverAssistanceIndicator(
    aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
    tap::communication::serial::RefSerial &refSerial,
    const Transform &worldToTurretTransform)
    : HudIndicator(refSerialTransmitter),
      visionCoprocessor(visionCoprocessor),
      refSerial(refSerial),
      worldToCameraTransform(worldToTurretTransform)
{
}

void DriverAssistanceIndicator::initialize()
{
    configureGraphic(GraphicIndex::TARGET, Tx::GraphicColor::GREEN);

    configureGraphic(GraphicIndex::HERO_TRACER, Tx::GraphicColor::ORANGE);
    configureGraphic(GraphicIndex::STANDARD_TRACER, Tx::GraphicColor::ORANGE);
    configureGraphic(GraphicIndex::SENTRY_TRACER, Tx::GraphicColor::ORANGE);

    configureGraphic(GraphicIndex::HERO_HP, Tx::GraphicColor::PURPLISH_RED);
    configureGraphic(GraphicIndex::STANDARD_HP, Tx::GraphicColor::PURPLISH_RED);
    configureGraphic(GraphicIndex::SENTRY_HP, Tx::GraphicColor::PURPLISH_RED);

    uint8_t graphicName[3];
    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &justALine.graphicData[0],
        graphicName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::PURPLISH_RED);
}

modm::ResumableResult<void> DriverAssistanceIndicator::sendInitialGraphics()
{
    RF_BEGIN(0);

    // Send the graphics
    RF_CALL(refSerialTransmitter.sendGraphic(&graphic));
    RF_CALL(refSerialTransmitter.sendGraphic(&justALine));

    RF_END();
}

modm::ResumableResult<void> DriverAssistanceIndicator::update()
{
    aimData = visionCoprocessor.getLastAimData(0);
    // bool visionHasTarget = visionCoprocessor.getSomeTurretHasTarget();

    if (!visionHasTarget)
    {
        deleteGraphic(GraphicIndex::TARGET);
    }
    else
    {
        // Get position
        Position enemyPlatePosition =
            Position(aimData.pva.xPos, aimData.pva.yPos, aimData.pva.zPos);

        // Draw the target box
        drawPlateTargetBox(enemyPlatePosition, GraphicIndex::TARGET);
    }

    // robotOrbits = visionCoprocessor.getLastRobotOrbitData();
    // hasStandard = false;
    // hasHero = false;
    // hasSentry = false;
    // for (int i = 0; i < visionCoprocessor.MAX_NUM_ROBOT_ORBITS; i++)
    // {
    //     int ID = robotOrbits.data[i].robotType;
    //     if (ID == 0) continue;

    //     Position robotOrbit =
    //         Position(robotOrbits.data[i].x, robotOrbits.data[i].y, robotOrbits.data[i].z);
    //     if (ID == 1)
    //     {
    //         hasHero = true;
    //         drawTracerLineToOrbit(robotOrbit, GraphicIndex::HERO_TRACER);
    //         drawHealthBarToOrbit(robotOrbit, GraphicIndex::HERO_HP, ID);
    //     }
    //     else if (ID == 3 || ID == 4)
    //     {
    //         hasStandard = true;
    //         drawTracerLineToOrbit(robotOrbit, GraphicIndex::STANDARD_TRACER);
    //         drawHealthBarToOrbit(robotOrbit, GraphicIndex::STANDARD_HP, ID);
    //     }
    //     else if (ID == 7)
    //     {
    //         hasSentry = true;
    //         drawTracerLineToOrbit(robotOrbit, GraphicIndex::SENTRY_TRACER);
    //         drawHealthBarToOrbit(robotOrbit, GraphicIndex::SENTRY_HP, ID);
    //     }
    // }

    // if (!hasHero)
    // {
    //     deleteGraphic(GraphicIndex::HERO_TRACER);
    //     deleteGraphic(GraphicIndex::HERO_HP);
    // }
    // if (!hasStandard)
    // {
    //     deleteGraphic(GraphicIndex::STANDARD_TRACER);
    //     deleteGraphic(GraphicIndex::STANDARD_HP);
    // }
    // if (!hasSentry)
    // {
    //     deleteGraphic(GraphicIndex::SENTRY_TRACER);
    //     deleteGraphic(GraphicIndex::SENTRY_HP);
    // }

    // Draw a line from (900, 300) to (1200, 600)
    RefSerialTransmitter::configLine(
        4,
        TRACER_LINE_ORIGIN.x,
        TRACER_LINE_ORIGIN.y,
        1200,
        600,
        &justALine.graphicData[0]);

    // Send the graphics
    RF_BEGIN(1);
    RF_CALL(refSerialTransmitter.sendGraphic(&graphic));
    RF_CALL(refSerialTransmitter.sendGraphic(&justALine));
    RF_END();
}

void DriverAssistanceIndicator::drawTracerLineToOrbit(Position orbit, GraphicIndex index)
{
    Position cameraFrameOrbit = worldToCameraTransform.apply(orbit);
    ProjectedResult screenFrameOrbit =
        convertCameraFrameToScreenFrame(cameraFrameOrbit + TRACER_LINE_OFFSET);

    auto graphicToModify = &graphic.graphicData[static_cast<uint8_t>(index)];
    graphicToModify->operation =
        graphicToModify->operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD : Tx::GRAPHIC_MODIFY;

    RefSerialTransmitter::configLine(
        1,
        TRACER_LINE_ORIGIN.x,
        TRACER_LINE_ORIGIN.y,
        screenFrameOrbit.screenX,
        screenFrameOrbit.screenY,
        graphicToModify);
};

void DriverAssistanceIndicator::drawHealthBarToOrbit(Position orbit, GraphicIndex index, int ID)
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

    RefSerialTransmitter::configInteger(
        20,
        3,
        screenFrameOrbit.screenX,
        screenFrameOrbit.screenY,
        robotHP,
        graphicToModify);
}

void DriverAssistanceIndicator::drawPlateTargetBox(Position orbit, GraphicIndex index)
{
    Position cameraFrameOrbit = worldToCameraTransform.apply(orbit);

    ProjectedResult screenFrameTopRight =
        convertCameraFrameToScreenFrame(cameraFrameOrbit + PLATE_CORNER_OFFSET);
    ProjectedResult screenFrameBottomLeft =
        convertCameraFrameToScreenFrame(cameraFrameOrbit - PLATE_CORNER_OFFSET);
    auto graphicToModify = &graphic.graphicData[static_cast<uint8_t>(index)];

    graphicToModify->operation =
        graphicToModify->operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD : Tx::GRAPHIC_MODIFY;

    RefSerialTransmitter::configRectangle(
        3,
        screenFrameBottomLeft.screenX,
        screenFrameBottomLeft.screenY,
        screenFrameTopRight.screenX,
        screenFrameTopRight.screenY,
        graphicToModify);
}

void DriverAssistanceIndicator::configureGraphic(GraphicIndex index, Tx::GraphicColor color)
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

void DriverAssistanceIndicator::deleteGraphic(GraphicIndex index)
{
    uint8_t idx = static_cast<uint8_t>(index);
    graphic.graphicData[idx].operation = Tx::GRAPHIC_DELETE;
}

}  // namespace aruwsrc::control::client_display
