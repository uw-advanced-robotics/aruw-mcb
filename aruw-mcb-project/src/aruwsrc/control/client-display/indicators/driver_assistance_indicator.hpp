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

#ifndef DRIVER_ASSISTANCE_INDICATOR_HPP_
#define DRIVER_ASSISTANCE_INDICATOR_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "../projection_utils.hpp"
#include "aruwsrc/algorithms/odometry/transformer_interface.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display
{
using namespace aruwsrc::algorithms::transforms;
using namespace tap::communication::serial;
/**
 * Draws a bounding box around the plate of where the vision system tells us to target.
 * Draws bars above robots to indicate the HP of the target.
 * Draws a line to the target.
 */
class DriverAssistanceIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    DriverAssistanceIndicator(
        aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
        RefSerialTransmitter &refSerialTransmitter,
        RefSerial &refSerial,
        const Transform &worldToTurretTransform);

    void initialize() override final;

    modm::ResumableResult<void> update() override final;

private:
    aruwsrc::serial::VisionCoprocessor &visionCoprocessor;
    RefSerial &refSerial;
    const Transform &worldToCameraTransform;

    bool visionHasTarget = false;


    /**
     * Useful utils I'll need.
     * Graphic -> ID enum
     * drawTracerLineToOrbit(graphic, position)
     * drawHealthBarToOrbit(graphic, position)
     *
     * drawTargetBoundingBox(graphic, position)
     */

    Tx::Graphic7Message graphic;

    Tx::Graphic7Message justALine;

    enum GraphicIndex : uint8_t
    {
        TARGET = 0,
        HERO_TRACER = 1,
        HERO_HP = 2,
        STANDARD_TRACER = 3,
        STANDARD_HP = 4,
        SENTRY_HP = 5,
        SENTRY_TRACER = 6
    };

    void deleteGraphic(GraphicIndex index)
    {
        uint8_t idx = static_cast<uint8_t>(index);
        graphic.graphicData[idx].operation = Tx::GRAPHIC_DELETE;
    }

    void configureGraphic(GraphicIndex index, Tx::GraphicColor color)
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

    Vector TRACER_LINE_OFFSET = Vector(0, 0, -0.3);
    modm::Vector2i TRACER_LINE_ORIGIN = modm::Vector2i(SCREEN_WIDTH / 2, 300);
    void drawTracerLineToOrbit(Position orbit, GraphicIndex index);

    Vector HEALTH_BAR_OFFSET = Vector(0, 0, 0.5);
    void drawHealthBarToOrbit(Position orbit, GraphicIndex index, int ID);

    static constexpr float SMALL_PLATE_LENGTH_M = 0.135;
    const Vector PLATE_CORNER_OFFSET =
        Vector(0, SMALL_PLATE_LENGTH_M / 2, SMALL_PLATE_LENGTH_M / 2);
    void drawPlateTargetBox(Position orbit, GraphicIndex index);
};

}  // namespace aruwsrc::control::client_display

#endif  // DRIVER_ASSISTANCE_INDICATOR_HPP_
