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

#ifndef VISION_ASSISTANCE_INDICATOR_HPP_
#define VISION_ASSISTANCE_INDICATOR_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "../projection_utils.hpp"
#include "aruwsrc/algorithms/odometry/transformer_interface.hpp"
#include "aruwsrc/communication/inter_robot_comm/inter_robot_transmitter.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display
{
using namespace aruwsrc::algorithms::transforms;
using namespace tap::communication::serial;
using namespace aruwsrc::communication::inter_robot_comm;
/**
 * Draws a bounding box around the plate of where the vision system tells us to target.
 * Draws bars above robots to indicate the HP of the target.
 * Draws a line to the target.
 */
class VisionAssistanceIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    VisionAssistanceIndicator(
        aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
        RefSerialTransmitter &refSerialTransmitter,
        RefSerial &refSerial,
        const Transform &worldToCameraTransform,
        InterRobotTransmitter &interRobotTransmitter);

    void initialize() override;

    modm::ResumableResult<void> update() override;

    modm::ResumableResult<void> sendInitialGraphics() override;

private:
    aruwsrc::serial::VisionCoprocessor &visionCoprocessor;
    RefSerial &refSerial;
    const Transform &worldToCameraTransform;
    InterRobotTransmitter &interRobotTransmitter;

    Tx::Graphic7Message graphic;

    enum GraphicIndex : uint8_t
    {
        TARGET = 0,
        HERO_TRACER = 1,
        HERO_HP = 2,
        STANDARD_TRACER = 3,
        STANDARD_HP = 4,
        SENTRY_TRACER = 5,
        SENTRY_HP = 6,
    };

    void deleteGraphic(GraphicIndex index);
    void configureGraphic(GraphicIndex index, Tx::GraphicColor color);

    Vector TRACER_LINE_OFFSET = Vector(0, 0, -0.3);
    modm::Vector2i TRACER_LINE_ORIGIN = modm::Vector2i(SCREEN_WIDTH / 2, 300);
    void drawTracerLineToOrbit(Position orbit, GraphicIndex index);

    Vector HEALTH_BAR_OFFSET = Vector(0, 0, 0.5);
    void drawHealthBarToOrbit(Position orbit, GraphicIndex index, int ID);

    static constexpr float SMALL_PLATE_LENGTH_M = 0.135;
    const Vector PLATE_CORNER_OFFSET =
        Vector(0, SMALL_PLATE_LENGTH_M / 2, SMALL_PLATE_LENGTH_M / 2);
    void drawPlateTargetBox();

    uint32_t TIME_CUTOFF_MS = 3000;
};

}  // namespace aruwsrc::control::client_display

#endif  // VISION_ASSISTANCE_INDICATOR_HPP_
