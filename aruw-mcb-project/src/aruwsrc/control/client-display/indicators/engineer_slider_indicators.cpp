/*
 * Copyright (c) 2026-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "engineer_slider_indicators.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display::indicators
{
EngineerSliderIndicators::EngineerSliderIndicators(
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
    const joint::JointSubsystem &extension,
    const cube_storage::CubeStorageSubsystem &cubeStorageSubsystem,
    AruwPressureSensor &wristSensor,
    AruwPressureSensor &cubeStorageSensor1,
    AruwPressureSensor &cubeStorageSensor2)
    : HudIndicator(refSerialTransmitter),
      extension(extension),
      cubeStorageSubsystem(cubeStorageSubsystem),
      wristSensor(wristSensor),
      cubeStorageSensor1(cubeStorageSensor1),
      cubeStorageSensor2(cubeStorageSensor2)
{
}

modm::ResumableResult<void> EngineerSliderIndicators::sendInitialGraphics()
{
    RF_BEGIN(0);
    RF_CALL(refSerialTransmitter.sendGraphic(&sliders));
    RF_END();
}

modm::ResumableResult<void> EngineerSliderIndicators::update()
{
    // Update extension
    float extensionPosition;
    float extensionPositionPercent;
    uint16_t extensionCircleX;

    // Update cube storage
    float cubeStoragePosition;
    float cubeStoragePositionPercent;
    uint16_t cubeStorage1X;
    uint16_t cubeStorage2X;

    RF_BEGIN(1);
    extensionPosition = extension.getPosition();
    extensionPositionPercent =
        percentage(extensionPosition, extension.getLowerBound(), extension.getUpperBound());
    extensionCircleX = EXTENSION_START_X + EXTENSION_WIDTH * extensionPositionPercent;

    cubeStoragePosition = cubeStorageSubsystem.getPosition();
    cubeStoragePositionPercent = percentage(
        cubeStoragePosition,
        cubeStorageSubsystem.getLowerBound(),
        cubeStorageSubsystem.getUpperBound());
    cubeStorage1X = CUBE_STORAGE_START_X +
                    CUBE_STORAGE_WIDTH *
                        std::fmod(cubeStoragePositionPercent + CUBE_STORAGE_OFFSET_PERCENT_1, 1.0f);
    cubeStorage2X = CUBE_STORAGE_START_X +
                    CUBE_STORAGE_WIDTH *
                        std::fmod(cubeStoragePositionPercent + CUBE_STORAGE_OFFSET_PERCENT_2, 1.0f);

    // Update cube storage and wrist graphic colors based on whether pressure sensor states
    sliders.graphicData[static_cast<uint8_t>(GraphicType::CUBE_STORAGE_WRIST)].color =
        static_cast<uint8_t>(
            wristSensor.isOnline()
                ? (wristSensor.getPressurekPascals() > 0 ? HAS_CUBE_COLOR : NO_CUBE_COLOR)
                : DISCONNECTED_COLOR);

    sliders.graphicData[static_cast<uint8_t>(GraphicType::CUBE_STORAGE_1)].color =
        static_cast<uint8_t>(
            cubeStorageSensor1.isOnline()
                ? (cubeStorageSensor1.getPressurekPascals() > 0 ? HAS_CUBE_COLOR : NO_CUBE_COLOR)
                : DISCONNECTED_COLOR);
    sliders.graphicData[static_cast<uint8_t>(GraphicType::CUBE_STORAGE_2)].color =
        static_cast<uint8_t>(
            cubeStorageSensor2.isOnline()
                ? (cubeStorageSensor2.getPressurekPascals() > 0 ? HAS_CUBE_COLOR : NO_CUBE_COLOR)
                : DISCONNECTED_COLOR);

    // Update circle positions
    RefSerialTransmitter::configCircle(
        CIRCLE_SIZE,
        extensionCircleX,
        EXTENSION_Y,
        CIRCLE_SIZE,
        &sliders.graphicData[static_cast<uint8_t>(GraphicType::EXTENSION_CIRCLE)]);
    RefSerialTransmitter::configCircle(
        CIRCLE_SIZE,
        cubeStorage1X,
        CUBE_STORAGE_Y,
        CIRCLE_SIZE,
        &sliders.graphicData[static_cast<uint8_t>(GraphicType::CUBE_STORAGE_1)]);
    RefSerialTransmitter::configCircle(
        CIRCLE_SIZE,
        cubeStorage2X,
        CUBE_STORAGE_Y,
        CIRCLE_SIZE,
        &sliders.graphicData[static_cast<uint8_t>(GraphicType::CUBE_STORAGE_2)]);

    RF_CALL(refSerialTransmitter.sendGraphic(&sliders));
    RF_END();
}

void EngineerSliderIndicators::initialize()
{
    uint8_t graphicName[3];
    getUnusedGraphicName(graphicName);

    RefSerialTransmitter::configGraphicGenerics(
        &sliders.graphicData[0],
        graphicName,
        Tx::GraphicOperation::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        LINE_COLOR);

    RefSerialTransmitter::configLine(
        LINE_WIDTH,
        EXTENSION_START_X,
        EXTENSION_Y - LINE_WIDTH / 2,
        EXTENSION_END_X,
        EXTENSION_Y + LINE_WIDTH / 2,
        &sliders.graphicData[static_cast<uint8_t>(GraphicType::EXTENSION_LINE)]);

    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &sliders.graphicData[static_cast<uint8_t>(GraphicType::EXTENSION_CIRCLE)],
        graphicName,
        Tx::GraphicOperation::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        EXTENSION_CIRCLE_COLOR);

    RefSerialTransmitter::configCircle(
        CIRCLE_SIZE,
        EXTENSION_START_X,
        EXTENSION_Y,
        CIRCLE_SIZE,
        &sliders.graphicData[static_cast<uint8_t>(GraphicType::EXTENSION_CIRCLE)]);

    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &sliders.graphicData[static_cast<uint8_t>(GraphicType::CUBE_STORAGE_LINE)],
        graphicName,
        Tx::GraphicOperation::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        LINE_COLOR);

    RefSerialTransmitter::configLine(
        LINE_WIDTH,
        CUBE_STORAGE_START_X,
        CUBE_STORAGE_Y - LINE_WIDTH / 2,
        CUBE_STORAGE_END_X,
        CUBE_STORAGE_Y + LINE_WIDTH / 2,
        &sliders.graphicData[static_cast<uint8_t>(GraphicType::CUBE_STORAGE_LINE)]);

    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &sliders.graphicData[static_cast<uint8_t>(GraphicType::CUBE_STORAGE_WRIST)],
        graphicName,
        Tx::GraphicOperation::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        NO_CUBE_COLOR);

    RefSerialTransmitter::configLine(
        LINE_WIDTH,
        EXTENSION_MIDDLE_X,
        CUBE_STORAGE_Y + CIRCLE_SIZE / 2,
        EXTENSION_MIDDLE_X,
        CUBE_STORAGE_Y + CIRCLE_SIZE * 3 / 2,
        &sliders.graphicData[static_cast<uint8_t>(GraphicType::CUBE_STORAGE_WRIST)]);

    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &sliders.graphicData[static_cast<uint8_t>(GraphicType::CUBE_STORAGE_1)],
        graphicName,
        Tx::GraphicOperation::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        NO_CUBE_COLOR);

    RefSerialTransmitter::configCircle(
        CIRCLE_SIZE,
        CUBE_STORAGE_START_X,
        CUBE_STORAGE_Y,
        CIRCLE_SIZE,
        &sliders.graphicData[static_cast<uint8_t>(GraphicType::CUBE_STORAGE_1)]);

    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &sliders.graphicData[static_cast<uint8_t>(GraphicType::CUBE_STORAGE_2)],
        graphicName,
        Tx::GraphicOperation::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        NO_CUBE_COLOR);

    RefSerialTransmitter::configCircle(
        CIRCLE_SIZE,
        CUBE_STORAGE_END_X,
        CUBE_STORAGE_Y,
        CIRCLE_SIZE,
        &sliders.graphicData[static_cast<uint8_t>(GraphicType::CUBE_STORAGE_2)]);
}

}  // namespace aruwsrc::control::client_display::indicators
