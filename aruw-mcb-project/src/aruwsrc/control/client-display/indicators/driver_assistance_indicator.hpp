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

using namespace aruwsrc::algorithms::transforms;

namespace aruwsrc::control::client_display
{
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
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        const Transform &worldToTurretTransform);

    void initialize() override final;

    modm::ResumableResult<void> update() override final;

    struct ProjectedPlateResult
    {
        bool inFrame;
        uint32_t bottomLeftX;
        uint32_t bottomLeftY;
        uint32_t topRightX;
        uint32_t topRightY;
        Position enemyCenterCameraFrame;

        ProjectedPlateResult()
            : inFrame(false),
              bottomLeftX(0),
              bottomLeftY(0),
              topRightX(0),
              topRightY(0),
              enemyCenterCameraFrame(0, 0, 0)
        {
        }
    };

private:
    aruwsrc::serial::VisionCoprocessor &visionCoprocessor;
    const Transform &worldToCameraTransform;

    Tx::Graphic1Message visionTargetGraphic;
    static constexpr uint16_t INDICATOR_LINE_THICKNESS = 3;

    RefSerialData::Tx::GraphicColor INDICATOR_COLOR = RefSerialData::Tx::GraphicColor::GREEN;

    static constexpr float SMALL_PLATE_LENGTH_M = 0.135;
    const Vector PLATE_CORNER_OFFSET =
        Vector(0, SMALL_PLATE_LENGTH_M / 2, SMALL_PLATE_LENGTH_M / 2);

    // In world frame
    Position enemyPosition;
    ProjectedPlateResult enemyPositionScreenFrame;

    ProjectedPlateResult getEnemyPlatePosition(Position &enemyPosition);

    /**
     * Useful utils I'll need.
     * Graphic -> ID enum
     * drawTracerLineToOrbit(graphic, position)
     * drawHealthBarToOrbit(graphic, position)
     *
     * drawTargetBoundingBox(graphic, position)
     */

    Tx::Graphic7Message graphic;

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
        graphic.graphicData[idx].operation == Tx::GRAPHIC_DELETE;
    }

    Vector TRACER_LINE_OFFSET = Vector(0, 0, -0.3);
    void drawTracerLineToOrbit(Position orbit, GraphicIndex index)
    {
        Position cameraFrameOrbit = worldToCameraTransform.apply(orbit);
        ProjectedResult screenFrameOrbit =
            convertCameraFrameToScreenFrame(cameraFrameOrbit - TRACER_LINE_OFFSET);

        
    }
};

}  // namespace aruwsrc::control::client_display

#endif  // DRIVER_ASSISTANCE_INDICATOR_HPP_
