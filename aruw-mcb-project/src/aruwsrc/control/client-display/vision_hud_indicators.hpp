/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef VISION_HUD_INDICATORS_HPP_
#define VISION_HUD_INDICATORS_HPP_

#include <optional>

#include "tap/architecture/timeout.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"
#include "reticle_indicator.hpp"

namespace aruwsrc::control::client_display
{
/**
 * Draws two squares next to the reticle when vision has a target acquired and otherwise hides the
 * squares.
 */
class VisionHudIndicators : public HudIndicator, protected modm::Resumable<3>
{
public:
    /**
     * Construct a VisionHudIndicators object.
     *
     * @param[in] visionCoprocessor visionCoprocessor instance.
     */
    VisionHudIndicators(
        aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

    void initialize() override final;

private:
    /** width/height (in pixels) of the square "vision has target" indicators to the left/right of
     * the hud center. */
    static constexpr int VISION_TARGET_FOUND_SQUARE_WIDTH = 15;
    /** The x location (in pixels) from the center of the reticle where the vision target found
     * squares are located. */
    static constexpr int VISION_TARGET_FOUND_X_DISTANCE_FROM_CENTER = 100;
    /** The y location (in pixels) where the vision target found squares are located. */
    static constexpr int VISION_TARGET_FOUND_Y_LOCATION = 425;
    /** The color of the vision target found squares. */
    static constexpr Tx::GraphicColor VISION_TARGET_FOUND_COLOR = Tx::GraphicColor::GREEN;
    /** The maximum refresh rate of the vision target found squares. */
    static constexpr uint32_t VISION_TARGET_FOUND_MAX_REFRESH_RATE = 250;

    aruwsrc::serial::VisionCoprocessor &visionCoprocessor;

    Tx::Graphic2Message visionTargetFoundGraphics;

    tap::arch::MilliTimeout updateVisionTargetFoundTimeout;

    std::optional<Tx::GraphicColor> prevVisionIndicatorColor = std::nullopt;
    std::optional<Tx::GraphicColor> newVisionIndicatorColor = std::nullopt;

    modm::ResumableResult<bool> updateVisionTargetStatus();

    /**
     * Initialize some vision hud indicator (a little square) with some x pixel location
     * (xBoxLocation), relative to the leftmost side of the screen.
     */
    void initializeVisionHudIndicator(Tx::GraphicData *graphicData, int xBoxLocation);
};
}  // namespace aruwsrc::control::client_display

#endif  // VISION_HUD_INDICATORS_HPP_
