/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "sliders_indicator.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
SlidersIndicator::SlidersIndicator(
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
    LinearSetpointInterface &gantryLift,
    LinearSetpointInterface &gantryExtension,
    LinearSetpointInterface &cubeLift,
    WristSubsystem &wristSubsystem,
    WristConfig wristConfig)
    : HudIndicator(refSerialTransmitter),
      gantryLift(gantryLift),
      gantryExtension(gantryExtension),
      cubeLift(cubeLift),
      wristSubsystem(wristSubsystem),
      wristConfig(wristConfig)
{
}

void SlidersIndicator::initialize()
{
    uint8_t graphicName[3];
    for (int i = 0; i < NUM_GRAPHICS; i++)
    {
        getUnusedGraphicName(graphicName);
        Tx::GraphicData *boundingBoxGraphicData = &sliderOutside.graphicData[i];

        RefSerialTransmitter::configGraphicGenerics(
            &sliderOutside.graphicData[i],
            graphicName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            GRAPHIC_COLOR);
        RefSerialTransmitter::configRectangle(
            BOUNDING_BOX_LINE_WIDTH,
            START_X,
            START_Y + i * Y_INCREMENT,
            START_X + BOUNDING_BOX_WIDTH,
            START_Y + i * Y_INCREMENT + BOUNDING_BOX_HEIGHT,
            boundingBoxGraphicData);

        Tx::GraphicData *circleGraphicData = &sliderInside.graphicData[i];
        RefSerialTransmitter::configGraphicGenerics(
            &sliderInside.graphicData[i],
            graphicName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            GRAPHIC_COLOR);
        // Put circle in the middle of the bounding box
        RefSerialTransmitter::configCircle(
            CIRCLE_LINE_WIDTH,
            START_X + BOUNDING_BOX_WIDTH / 2,
            START_Y + i * Y_INCREMENT + BOUNDING_BOX_HEIGHT / 2,
            CIRCLE_SIZE,
            circleGraphicData);
    }
}

modm::ResumableResult<void> SlidersIndicator::sendInitialGraphics()
{
    RF_BEGIN(0);
    RF_CALL(refSerialTransmitter.sendGraphic(&sliderOutside));
    RF_CALL(refSerialTransmitter.sendGraphic(&sliderInside));
    RF_END();
}

modm::ResumableResult<void> SlidersIndicator::update()
{
    // Update gantry lift
    float gantryLiftPosition = gantryLift.getPosition();
    float gantryLiftPositionPercent =
        getPercentage(gantryLiftPosition, gantryLift.getLowerBound(), gantryLift.getUpperBound());
    uint16_t gantryLiftCircleX = START_X + BOUNDING_BOX_WIDTH * gantryLiftPositionPercent;

    // Update extension
    float extensionPosition = gantryExtension.getPosition();
    float extensionPositionPercent = getPercentage(
        extensionPosition,
        gantryExtension.getLowerBound(),
        gantryExtension.getUpperBound());
    uint16_t extensionCircleX = START_X + BOUNDING_BOX_WIDTH * extensionPositionPercent;

    // Update cube lift
    float cubeLiftSetpoint = cubeLift.getSetpoint();
    float cubeLiftPositionPercent;
    if (cubeLiftSetpoint == -40)
    {
        cubeLiftPositionPercent = 1 / 3;
    }
    else if (cubeLiftSetpoint == -220)
    {
        cubeLiftPositionPercent = 2 / 3;
    }
    else if (cubeLiftSetpoint == -310)
    {
        cubeLiftPositionPercent = 1;
    }
    else
    {
        cubeLiftPositionPercent = 0;
    }
    uint16_t cubeLiftCircleX = START_X + BOUNDING_BOX_WIDTH * cubeLiftPositionPercent;

    // Update wrist pitch
    float wristPitchPosition = wristSubsystem.getPitch();
    float wristPitchPositionPercent =
        getPercentage(wristPitchPosition, wristConfig.minPitch, wristConfig.maxPitch);
    uint16_t wristPitchCircleX = START_X + BOUNDING_BOX_WIDTH * wristPitchPositionPercent;

    // Update wrist yaw
    float wristYawPosition = wristSubsystem.getYaw();
    float wristYawPositionPercent =
        getPercentage(wristYawPosition, wristConfig.minYaw, wristConfig.maxYaw);
    uint16_t wristYawCircleX = START_X + BOUNDING_BOX_WIDTH * wristYawPositionPercent;

    RF_BEGIN(1);
    // Update gantry lift circle
    RefSerialTransmitter::configCircle(
        CIRCLE_LINE_WIDTH,
        gantryLiftCircleX,
        START_Y + Y_INCREMENT * static_cast<uint16_t>(GraphicType::GANTRY_LIFT) +
            BOUNDING_BOX_HEIGHT / 2,
        CIRCLE_SIZE,
        &sliderInside.graphicData[static_cast<uint8_t>(GraphicType::GANTRY_LIFT)]);

    // Update extension circle
    RefSerialTransmitter::configCircle(
        CIRCLE_LINE_WIDTH,
        extensionCircleX,
        START_Y + Y_INCREMENT * static_cast<uint16_t>(GraphicType::GANTRY_EXTENSION) +
            BOUNDING_BOX_HEIGHT / 2,
        CIRCLE_SIZE,
        &sliderInside.graphicData[static_cast<uint8_t>(GraphicType::GANTRY_EXTENSION)]);

    // Update cube lift circle
    RefSerialTransmitter::configCircle(
        CIRCLE_LINE_WIDTH,
        cubeLiftCircleX,
        START_Y + Y_INCREMENT * static_cast<uint16_t>(GraphicType::CUBE_LIFT) +
            BOUNDING_BOX_HEIGHT / 2,
        CIRCLE_SIZE,
        &sliderInside.graphicData[static_cast<uint8_t>(GraphicType::CUBE_LIFT)]);

    // Update wrist pitch circle
    RefSerialTransmitter::configCircle(
        CIRCLE_LINE_WIDTH,
        wristPitchCircleX,
        START_Y + Y_INCREMENT * static_cast<uint16_t>(GraphicType::WRIST_PITCH) +
            BOUNDING_BOX_HEIGHT / 2,
        CIRCLE_SIZE,
        &sliderInside.graphicData[static_cast<uint8_t>(GraphicType::WRIST_PITCH)]);

    // Update wrist yaw circle
    RefSerialTransmitter::configCircle(
        CIRCLE_LINE_WIDTH,
        wristYawCircleX,
        START_Y + Y_INCREMENT * static_cast<uint16_t>(GraphicType::WRIST_YAW) +
            BOUNDING_BOX_HEIGHT / 2,
        CIRCLE_SIZE,
        &sliderInside.graphicData[static_cast<uint8_t>(GraphicType::WRIST_YAW)]);

    RF_CALL(refSerialTransmitter.sendGraphic(&sliderInside));
    RF_END();
}

}  // namespace aruwsrc::control::client_display
